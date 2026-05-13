#include "infrared_host.h"
#include "can.h"

IR_Host_Context_t ir_host_context = {0};

typedef struct {
    uint8_t tx_data[8];
    uint8_t rx_data[8];
    uint8_t rx_data_len;
    uint32_t update_timestamp;
    uint32_t test_cycle;
    uint8_t task_state;
    uint8_t online_count;
    uint8_t total_modules;

    struct {
        uint8_t discovered;
        uint8_t online;
        uint8_t busy;
        uint8_t consecutive_errors;
        uint32_t total_rx_count;
        uint32_t last_rx_time;
    } mod1, mod2;

    struct {
        uint32_t tx_success_count;
        uint32_t tx_fail_count;
        uint32_t rx_valid_count;
        uint32_t rx_invalid_count;
        uint8_t data_match;
        uint8_t data_check_result;
    } stats;

    uint8_t data_changed_flag;

    uint32_t all_rx_frame_count;
    uint32_t ack_frame_count;
    uint32_t data_frame_count;
    uint32_t other_frame_count;
    uint32_t last_rx_can_id;
    uint8_t last_rx_dlc;
    uint8_t last_rx_data[8];
} IR_Debug_Data_t;

IR_Debug_Data_t ir_debug = {0};

static TaskHandle_t ir_host_task_handle = NULL;

/* ======================== 内部函数声明 (带__后缀) ======================== */

static void IR_ProcessRxQueue__(void);
static void IR_HandleAckNack__(uint8_t module_id, uint32_t id, uint8_t *data, uint8_t dlc);
static void IR_EnqueueRxFrame__(uint32_t can_id, uint8_t *data, uint8_t dlc);
static void IR_DiscoveryPhase__(void);
static void IR_PollModule__(IR_Module_Node_t *module);
static void IR_UpdateOnlineStatus__(void);
static void IR_TaskEntry__(void *argument);
static IR_Module_Node_t* IR_FindModule__(uint8_t module_id);
static IR_Module_Node_t* IR_GetModuleByIndex__(uint8_t index);
static void IR_TestTaskEntry__(void *argument);

/* ================================================================
 *          ★ 初始化 (不操作CAN硬件)
 * ================================================================ */

bool IR_Init(CAN_HandleTypeDef *hcan, uint8_t *module_ids, uint8_t count)
{
    if (hcan == NULL || module_ids == NULL || count == 0 || count > IR_HOST_MAX_MODULES) {
        return false;
    }

    /* 初始化上下文 (不操作CAN硬件) */
    ir_host_context.hcan = hcan;
    ir_host_context.rx_queue = xQueueCreate(IR_HOST_RX_QUEUE_SIZE, sizeof(IR_Host_RxFrame_t));
    ir_host_context.module_list.head = NULL;
    ir_host_context.module_list.count = 0;
    ir_host_context.module_list.mutex = xSemaphoreCreateMutex();
    ir_host_context.initialized = true;
    ir_host_context.task_state = IR_HOST_TASK_STATE_INIT;
    ir_host_context.current_poll_index = 0;
    ir_host_context.last_poll_time = 0;
    ir_host_context.discovery_start_time = 0;
    ir_host_context.can_timeout_ms    = IR_HOST_CAN_TIMEOUT_MS;
    ir_host_context.max_retry         = IR_HOST_MAX_RETRY_COUNT;
    ir_host_context.poll_interval_ms  = IR_HOST_POLL_INTERVAL_MS;
    ir_host_context.online_timeout_ms = IR_HOST_ONLINE_TIMEOUT_MS;

    /* 添加模块 */
    for (uint8_t i = 0; i < count; i++) {
        IR_AddModule(module_ids[i]);
    }

    /* 启动后台任务 */
    if (ir_host_task_handle == NULL) {
        xTaskCreate(IR_TaskEntry__, "IRHost", 1024, NULL, 3, &ir_host_task_handle);
    }

    return true;
}

/* ================================================================
 *          ★ 阻塞发送
 * ================================================================ */

bool IR_Send(uint8_t module_id, uint8_t *data, uint8_t length)
{
    return IR_SendRetry(module_id, data, length, ir_host_context.max_retry);
}

bool IR_SendRetry(uint8_t module_id, uint8_t *data, uint8_t length, uint8_t retry)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];

    if (ir_host_context.hcan == NULL) return false;

    IR_Module_Node_t *module = IR_FindModule__(module_id);
    if (module == NULL) return false;
    if (length > 8 || length == 0) return false;

    uint32_t timeout_ms = ir_host_context.can_timeout_ms;

    for (uint8_t r = 0; r < retry; r++) {
        while (module->busy) {
            IR_ProcessRxQueue__();
            vTaskDelay(pdMS_TO_TICKS(10));
        }

        module->status = IR_HOST_STATUS_IDLE;

        memset(tx_data, 0, 8);
        memcpy(tx_data, data, length);

        tx_header.StdId = IR_HOST_CAN_ID_DATA(module_id);
        tx_header.ExtId = 0;
        tx_header.IDE = CAN_ID_STD;
        tx_header.RTR = CAN_RTR_DATA;
        tx_header.DLC = length;
        tx_header.TransmitGlobalTime = DISABLE;

        module->busy = true;
        module->status = IR_HOST_STATUS_SENDING;

        if (HAL_CAN_AddTxMessage(ir_host_context.hcan, &tx_header, tx_data, &tx_mailbox) == HAL_OK) {
            module->last_tx_time = xTaskGetTickCount();

            uint32_t start = xTaskGetTickCount();
            while ((xTaskGetTickCount() - start) < timeout_ms) {
                IR_ProcessRxQueue__();
                if (!module->busy) {
                    if (module->status == IR_HOST_STATUS_SUCCESS) return true;
                    if (module->status == IR_HOST_STATUS_NACK) break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        } else {
            module->busy = false;
            module->status = IR_HOST_STATUS_ERROR;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    return false;
}

/* ================================================================
 *          ★ 非阻塞发送
 * ================================================================ */

bool IR_SendAsync(uint8_t module_id, uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];

    if (ir_host_context.hcan == NULL) return false;

    IR_Module_Node_t *module = IR_FindModule__(module_id);
    if (module == NULL) return false;
    if (module->busy) return false;
    if (length > 8 || length == 0) return false;

    uint32_t elapsed = xTaskGetTickCount() - module->last_tx_time;
    if (elapsed < IR_HOST_FRAME_INTERVAL_MS) return false;

    memset(tx_data, 0, 8);
    memcpy(tx_data, data, length);

    tx_header.StdId = IR_HOST_CAN_ID_DATA(module_id);
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = length;
    tx_header.TransmitGlobalTime = DISABLE;

    module->busy = true;
    module->status = IR_HOST_STATUS_SENDING;

    if (HAL_CAN_AddTxMessage(ir_host_context.hcan, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
        module->busy = false;
        module->status = IR_HOST_STATUS_ERROR;
        return false;
    }

    module->last_tx_time = xTaskGetTickCount();
    module->status = IR_HOST_STATUS_WAIT_ACK;
    return true;
}

/* ================================================================
 *          ★ 数据读取
 * ================================================================ */

bool IR_Read(uint8_t module_id, uint8_t *data, uint8_t *length)
{
    IR_Module_Node_t *module;

    /* 0xFF = 读取第一个有效模块 */
    if (module_id == 0xFF) {
        module = ir_host_context.module_list.head;
        while (module != NULL) {
            if (module->online && module->data_cache.valid) break;
            module = module->next;
        }
    } else {
        module = IR_FindModule__(module_id);
    }

    if (module == NULL || !module->data_cache.valid) return false;

    if (data != NULL) {
        memcpy(data, module->data_cache.raw_data, 8);
    }
    if (length != NULL) {
        *length = module->last_response.length > 8 ? 8 : module->last_response.length;
    }

    return true;
}

bool IR_IsOnline(uint8_t module_id)
{
    IR_Module_Node_t *module = IR_FindModule__(module_id);
    if (module == NULL) return false;
    return module->online;
}

/* ================================================================
 *          ★ CAN中断回调
 * ================================================================ */

void IR_OnCanRx(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    if (rx_header == NULL || rx_data == NULL) return;

    ir_debug.all_rx_frame_count++;
    ir_debug.last_rx_can_id = rx_header->StdId;
    ir_debug.last_rx_dlc = rx_header->DLC;
    memcpy(ir_debug.last_rx_data, rx_data, rx_header->DLC > 8 ? 8 : rx_header->DLC);

    if (IR_HOST_IS_ACK_FRAME(rx_header->StdId)) {
        ir_debug.ack_frame_count++;
    } else if (IR_HOST_IS_DATA_FRAME(rx_header->StdId)) {
        ir_debug.data_frame_count++;
    } else {
        ir_debug.other_frame_count++;
    }

    IR_EnqueueRxFrame__(rx_header->StdId, rx_data, rx_header->DLC);
}

static void IR_EnqueueRxFrame__(uint32_t can_id, uint8_t *data, uint8_t dlc)
{
    IR_Host_RxFrame_t rx_frame;

    rx_frame.can_id = can_id;
    rx_frame.module_id = IR_HOST_CAN_ID_GET_MODULE(can_id);
    rx_frame.dlc = dlc > 8 ? 8 : dlc;
    memcpy(rx_frame.data, data, rx_frame.dlc);
    rx_frame.timestamp = xTaskGetTickCountFromISR();

    if (ir_host_context.rx_queue != NULL) {
        xQueueSendFromISR(ir_host_context.rx_queue, &rx_frame, NULL);
    }
}

/* ================================================================
 *          辅助功能
 * ================================================================ */

IR_Data_CheckResult_t IR_CheckData(uint8_t module_id)
{
    IR_Module_Node_t *module = IR_FindModule__(module_id);

    if (module == NULL) return IR_DATA_CHECK_NO_MODULE;
    if (!module->online || !module->discovered) return IR_DATA_CHECK_OFFLINE;
    if (!module->data_cache.valid) return IR_DATA_CHECK_CRC_ERR;

    uint32_t elapsed = xTaskGetTickCount() - module->data_cache.update_timestamp;
    if (elapsed > IR_HOST_DATA_STALE_MS) return IR_DATA_CHECK_STALE;
    if (module->data_cache.consistent_count < IR_HOST_CONSISTENT_COUNT) return IR_DATA_CHECK_INCONSISTENT;

    return IR_DATA_CHECK_OK;
}

bool IR_Ping(uint8_t module_id)
{
    return IR_PingTimeout(module_id, ir_host_context.can_timeout_ms);
}

bool IR_Reset(uint8_t module_id)
{
    return IR_ResetTimeout(module_id, ir_host_context.can_timeout_ms);
}

bool IR_AddModule(uint8_t module_id)
{
    if (!ir_host_context.initialized) return false;
    if (ir_host_context.module_list.count >= IR_HOST_MAX_MODULES) return false;

    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);

    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        if (current->module_id == module_id) {
            xSemaphoreGive(ir_host_context.module_list.mutex);
            return false;
        }
        current = current->next;
    }

    IR_Module_Node_t *new_node = (IR_Module_Node_t *)pvPortMalloc(sizeof(IR_Module_Node_t));
    if (new_node == NULL) {
        xSemaphoreGive(ir_host_context.module_list.mutex);
        return false;
    }

    new_node->module_id = module_id;
    new_node->status = IR_HOST_STATUS_IDLE;
    new_node->online = false;
    new_node->busy = false;
    new_node->discovered = false;
    new_node->last_rx_time = 0;
    new_node->last_tx_time = xTaskGetTickCount() - IR_HOST_FRAME_INTERVAL_MS - 1;
    new_node->poll_fail_count = 0;
    new_node->consecutive_errors = 0;
    memset(&new_node->last_response, 0, sizeof(IR_Host_ResponseFrame_t));
    memset(&new_node->data_cache, 0, sizeof(IR_Module_DataCache_t));
    new_node->next = ir_host_context.module_list.head;
    ir_host_context.module_list.head = new_node;
    ir_host_context.module_list.count++;

    xSemaphoreGive(ir_host_context.module_list.mutex);
    return true;
}

bool IR_RemoveModule(uint8_t module_id)
{
    if (!ir_host_context.initialized) return false;

    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);

    IR_Module_Node_t **current = &ir_host_context.module_list.head;
    while (*current != NULL) {
        if ((*current)->module_id == module_id) {
            IR_Module_Node_t *to_remove = *current;
            *current = (*current)->next;
            vPortFree(to_remove);
            ir_host_context.module_list.count--;
            xSemaphoreGive(ir_host_context.module_list.mutex);
            return true;
        }
        current = &(*current)->next;
    }

    xSemaphoreGive(ir_host_context.module_list.mutex);
    return false;
}

uint8_t IR_GetOnlineCount(void)
{
    uint8_t count = 0;
    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        if (current->online && current->discovered) count++;
        current = current->next;
    }
    xSemaphoreGive(ir_host_context.module_list.mutex);
    return count;
}

void IR_ForceRediscover(void)
{
    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        current->discovered = false;
        current->online = false;
        current->poll_fail_count = 0;
        current->consecutive_errors = 0;
        memset(&current->data_cache, 0, sizeof(IR_Module_DataCache_t));
        current = current->next;
    }
    ir_host_context.task_state = IR_HOST_TASK_STATE_DISCOVERY;
    ir_host_context.discovery_start_time = xTaskGetTickCount();
    xSemaphoreGive(ir_host_context.module_list.mutex);
}

IR_Host_TaskState_t IR_GetTaskState(void)
{
    return ir_host_context.task_state;
}

uint8_t IR_GetModuleCount(void)
{
    return ir_host_context.module_list.count;
}

IR_Data_CheckResult_t IR_CheckAllModules(void)
{
    IR_Data_CheckResult_t worst = IR_DATA_CHECK_OK;
    bool found = false;
    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        found = true;
        IR_Data_CheckResult_t r = IR_CheckData(current->module_id);
        if (r > worst) worst = r;
        current = current->next;
    }
    return found ? worst : IR_DATA_CHECK_NO_MODULE;
}

/* ================================================================
 *          高级 API
 * ================================================================ */

bool IR_SendCommand(uint8_t module_id, IR_Host_Command_t cmd, uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];

    if (ir_host_context.hcan == NULL) return false;

    IR_Module_Node_t *module = IR_FindModule__(module_id);
    if (module == NULL) return false;
    if (module->busy) return false;
    if (length > 6) return false;

    uint32_t elapsed = xTaskGetTickCount() - module->last_tx_time;
    if (elapsed < IR_HOST_FRAME_INTERVAL_MS) return false;

    memset(tx_data, 0, 8);
    tx_data[0] = module_id;
    tx_data[1] = cmd;
    if (data != NULL && length > 0) {
        memcpy(&tx_data[2], data, length);
    }

    tx_header.StdId = IR_HOST_CAN_ID_COMMAND(module_id);
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8;
    tx_header.TransmitGlobalTime = DISABLE;

    module->busy = true;
    module->status = IR_HOST_STATUS_SENDING;

    if (HAL_CAN_AddTxMessage(ir_host_context.hcan, &tx_header, tx_data, &tx_mailbox) != HAL_OK) {
        module->busy = false;
        module->status = IR_HOST_STATUS_ERROR;
        return false;
    }

    module->last_tx_time = xTaskGetTickCount();
    module->status = IR_HOST_STATUS_WAIT_ACK;
    return true;
}

bool IR_PingTimeout(uint8_t module_id, uint32_t timeout_ms)
{
    IR_Module_Node_t *module = IR_FindModule__(module_id);
    if (module == NULL) return false;
    if (module->busy) return false;

    uint32_t elapsed = xTaskGetTickCount() - module->last_tx_time;
    if (elapsed < IR_HOST_FRAME_INTERVAL_MS) return false;

    if (!IR_SendCommand(module_id, IR_HOST_CMD_PING, NULL, 0)) return false;

    uint32_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < timeout_ms) {
        IR_ProcessRxQueue__();
        if (!module->busy) {
            if (module->status == IR_HOST_STATUS_SUCCESS) return true;
            if (module->status == IR_HOST_STATUS_ERROR ||
                module->status == IR_HOST_STATUS_NACK ||
                module->status == IR_HOST_STATUS_TIMEOUT) return false;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    module->busy = false;
    module->status = IR_HOST_STATUS_TIMEOUT;
    return false;
}

bool IR_ReadStatus(uint8_t module_id, uint8_t *status, uint32_t timeout_ms)
{
    IR_Module_Node_t *module = IR_FindModule__(module_id);
    if (module == NULL) return false;
    if (module->busy) return false;

    uint32_t elapsed = xTaskGetTickCount() - module->last_tx_time;
    if (elapsed < IR_HOST_FRAME_INTERVAL_MS) return false;

    if (!IR_SendCommand(module_id, IR_HOST_CMD_READ_STATUS, NULL, 0)) return false;

    uint32_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < timeout_ms) {
        IR_ProcessRxQueue__();
        if (!module->busy) {
            if (status != NULL && module->last_response.length > 0) {
                *status = module->last_response.data[0];
            }
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    module->busy = false;
    module->status = IR_HOST_STATUS_TIMEOUT;
    return false;
}

bool IR_ResetTimeout(uint8_t module_id, uint32_t timeout_ms)
{
    IR_Module_Node_t *module = IR_FindModule__(module_id);
    if (module == NULL) return false;
    if (module->busy) return false;

    uint32_t elapsed = xTaskGetTickCount() - module->last_tx_time;
    if (elapsed < IR_HOST_FRAME_INTERVAL_MS) return false;

    if (!IR_SendCommand(module_id, IR_HOST_CMD_RESET, NULL, 0)) return false;

    uint32_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < timeout_ms) {
        IR_ProcessRxQueue__();
        if (!module->busy) return true;
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    module->busy = false;
    module->status = IR_HOST_STATUS_TIMEOUT;
    return false;
}

/* ================================================================
 *          内部函数 (带__后缀, 不对外暴露)
 * ================================================================ */

static IR_Module_Node_t* IR_FindModule__(uint8_t module_id)
{
    if (!ir_host_context.initialized) return NULL;

    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
    IR_Module_Node_t *current = ir_host_context.module_list.head;
    while (current != NULL) {
        if (current->module_id == module_id) {
            xSemaphoreGive(ir_host_context.module_list.mutex);
            return current;
        }
        current = current->next;
    }
    xSemaphoreGive(ir_host_context.module_list.mutex);
    return NULL;
}

static IR_Module_Node_t* IR_GetModuleByIndex__(uint8_t index)
{
    if (!ir_host_context.initialized || index >= ir_host_context.module_list.count) return NULL;

    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
    IR_Module_Node_t *current = ir_host_context.module_list.head;
    for (uint8_t i = 0; i < index && current != NULL; i++) {
        current = current->next;
    }
    xSemaphoreGive(ir_host_context.module_list.mutex);
    return current;
}

static void IR_HandleAckNack__(uint8_t module_id, uint32_t id, uint8_t *data, uint8_t dlc)
{
    if (IR_HOST_IS_ACK_FRAME(id)) {
        if (dlc >= 2 && data[0] == IR_ACK_MAGIC && data[1] == IR_ACK_MAGIC) {
            IR_Module_Node_t *module = IR_FindModule__(module_id);
            if (module != NULL) {
                module->status = IR_HOST_STATUS_SUCCESS;
                module->busy = false;
                module->consecutive_errors = 0;
                module->online = true;
                module->discovered = true;
                module->last_rx_time = xTaskGetTickCount();
            }
        }
        else if (dlc >= 2 && data[0] == IR_NACK_MAGIC && data[1] == IR_NACK_MAGIC) {
            IR_Module_Node_t *module = IR_FindModule__(module_id);
            if (module != NULL) {
                module->status = IR_HOST_STATUS_NACK;
                module->busy = false;
            }
        }
    }
}

static void IR_ProcessRxQueue__(void)
{
    IR_Host_RxFrame_t rx_frame;
    while (xQueueReceive(ir_host_context.rx_queue, &rx_frame, 0) == pdTRUE) {
        if (IR_HOST_IS_ACK_FRAME(rx_frame.can_id)) {
            IR_HandleAckNack__(rx_frame.module_id, rx_frame.can_id, rx_frame.data, rx_frame.dlc);
        } else if (IR_HOST_IS_DATA_FRAME(rx_frame.can_id)) {
            IR_Module_Node_t *module = IR_FindModule__(rx_frame.module_id);
            if (module != NULL && rx_frame.dlc >= 1) {
                uint8_t data_len = rx_frame.dlc;
                module->data_cache.total_rx_count++;
                if (data_len > 0) {
                    bool data_changed = false;
                    if (module->data_cache.valid &&
                        data_len <= 8 &&
                        memcmp(module->data_cache.raw_data, rx_frame.data, data_len) != 0) {
                        data_changed = true;
                    }
                    memcpy(module->data_cache.raw_data, rx_frame.data, data_len > 8 ? 8 : data_len);
                    module->data_cache.update_timestamp = rx_frame.timestamp;
                    module->data_cache.valid = true;
                    if (!data_changed) {
                        if (module->data_cache.consistent_count < 255) module->data_cache.consistent_count++;
                    } else {
                        module->data_cache.consistent_count = 1;
                    }
                    module->last_response.module_id = rx_frame.module_id;
                    module->last_response.status = IR_HOST_STATUS_SUCCESS;
                    module->last_response.length = data_len;
                    module->last_response.timestamp = rx_frame.timestamp;
                    module->last_response.valid = true;
                    if (data_len > 0) memcpy(module->last_response.data, rx_frame.data, data_len);
                    module->last_rx_time = rx_frame.timestamp;
                    module->online = true;
                    module->busy = false;
                    module->discovered = true;
                    module->poll_fail_count = 0;
                    module->consecutive_errors = 0;
                }
            }
        }
    }
}

static void IR_DiscoveryPhase__(void)
{
    static uint8_t discover_index = 0;
    uint32_t now = xTaskGetTickCount();

    if (now - ir_host_context.discovery_start_time > 5000) {
        ir_host_context.task_state = IR_HOST_TASK_STATE_RUNNING;
        discover_index = 0;
        return;
    }

    IR_Module_Node_t *module = IR_GetModuleByIndex__(discover_index);
    if (module != NULL && !module->discovered) {
        if (!module->busy && (now - module->last_tx_time) > ir_host_context.poll_interval_ms) {
            if (IR_PingTimeout(module->module_id, 200)) {
                module->discovered = true;
                module->online = true;
                module->last_rx_time = xTaskGetTickCount();
            } else {
                module->poll_fail_count++;
            }
        }
    }

    discover_index++;
    if (discover_index >= ir_host_context.module_list.count) discover_index = 0;

    vTaskDelay(pdMS_TO_TICKS(50));
}

static void IR_PollModule__(IR_Module_Node_t *module)
{
    if (module == NULL || module->busy) return;

    uint32_t now = xTaskGetTickCount();

    if (!module->online || !module->discovered) {
        if (now - module->last_tx_time > ir_host_context.poll_interval_ms * 2) {
            if (IR_PingTimeout(module->module_id, 300)) {
                module->discovered = true;
                module->online = true;
                module->last_rx_time = xTaskGetTickCount();
            } else {
                module->poll_fail_count++;
                module->consecutive_errors++;
                if (module->consecutive_errors > 10) {
                    module->online = false;
                    module->discovered = false;
                    module->data_cache.valid = false;
                }
            }
        }
    } else {
        /* 如果最近收到过数据帧, 说明从机在线, 不需要再主动轮询 */
        if (now - module->last_rx_time < ir_host_context.poll_interval_ms * 2) {
            return;
        }
        if (now - module->last_tx_time >= ir_host_context.poll_interval_ms) {
            if (IR_ReadStatus(module->module_id, NULL, 300)) {
                module->consecutive_errors = 0;
                module->last_rx_time = xTaskGetTickCount();
            } else {
                module->poll_fail_count++;
                module->consecutive_errors++;
                if (module->consecutive_errors > 5) {
                    module->online = false;
                    module->data_cache.valid = false;
                }
            }
        }
    }
}

static void IR_UpdateOnlineStatus__(void)
{
    uint32_t now = xTaskGetTickCount();

    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
    IR_Module_Node_t *current = ir_host_context.module_list.head;

    while (current != NULL) {
        if (current->online && (now - current->last_rx_time) > ir_host_context.online_timeout_ms) {
            current->online = false;
            current->data_cache.valid = false;
        }
        if (current->data_cache.valid && (now - current->data_cache.update_timestamp) > IR_HOST_DATA_STALE_MS * 3) {
            current->data_cache.valid = false;
        }
        current = current->next;
    }
    xSemaphoreGive(ir_host_context.module_list.mutex);
}

static void IR_TaskEntry__(void *argument)
{
    IR_Host_RxFrame_t rx_frame;
    (void)argument;

    ir_host_context.task_state = IR_HOST_TASK_STATE_DISCOVERY;
    ir_host_context.discovery_start_time = xTaskGetTickCount();
    ir_host_context.current_poll_index = 0;
    ir_host_context.last_poll_time = 0;

    for (;;) {
        while (xQueueReceive(ir_host_context.rx_queue, &rx_frame, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (IR_HOST_IS_ACK_FRAME(rx_frame.can_id)) {
                IR_HandleAckNack__(rx_frame.module_id, rx_frame.can_id, rx_frame.data, rx_frame.dlc);
            }
            else if (IR_HOST_IS_DATA_FRAME(rx_frame.can_id)) {
                IR_Module_Node_t *module = IR_FindModule__(rx_frame.module_id);
                if (module != NULL && rx_frame.dlc >= 1) {
                    uint8_t data_len = rx_frame.dlc;
                    module->data_cache.total_rx_count++;
                    if (data_len > 0) {
                        bool data_changed = false;
                        if (module->data_cache.valid && data_len <= 8 &&
                            memcmp(module->data_cache.raw_data, rx_frame.data, data_len) != 0) {
                            data_changed = true;
                        }
                        memcpy(module->data_cache.raw_data, rx_frame.data, data_len > 8 ? 8 : data_len);
                        module->data_cache.update_timestamp = rx_frame.timestamp;
                        module->data_cache.valid = true;
                        if (!data_changed) {
                            if (module->data_cache.consistent_count < 255) module->data_cache.consistent_count++;
                        } else {
                            module->data_cache.consistent_count = 1;
                        }
                        module->last_response.module_id = rx_frame.module_id;
                        module->last_response.status = IR_HOST_STATUS_SUCCESS;
                        module->last_response.length = data_len;
                        module->last_response.timestamp = rx_frame.timestamp;
                        module->last_response.valid = true;
                        if (data_len > 0) memcpy(module->last_response.data, rx_frame.data, data_len);
                        module->last_rx_time = rx_frame.timestamp;
                        module->online = true;
                        module->busy = false;
                        module->discovered = true;
                        module->poll_fail_count = 0;
                        module->consecutive_errors = 0;
                    }
                }
            }
        }

        switch (ir_host_context.task_state) {
            case IR_HOST_TASK_STATE_DISCOVERY:
                IR_DiscoveryPhase__();
                break;

            case IR_HOST_TASK_STATE_RUNNING:
            {
                uint32_t now = xTaskGetTickCount();
                if (now - ir_host_context.last_poll_time >= ir_host_context.poll_interval_ms) {
                    ir_host_context.last_poll_time = now;
                    IR_Module_Node_t *module = IR_GetModuleByIndex__(ir_host_context.current_poll_index);
                    if (module != NULL) IR_PollModule__(module);
                    ir_host_context.current_poll_index++;
                    if (ir_host_context.current_poll_index >= ir_host_context.module_list.count) {
                        ir_host_context.current_poll_index = 0;
                        IR_UpdateOnlineStatus__();
                    }
                } else {
                    vTaskDelay(pdMS_TO_TICKS(10));
                }
                break;
            }

            case IR_HOST_TASK_STATE_ERROR:
                vTaskDelay(pdMS_TO_TICKS(100));
                break;

            default:
                ir_host_context.task_state = IR_HOST_TASK_STATE_DISCOVERY;
                ir_host_context.discovery_start_time = xTaskGetTickCount();
                break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

/* ================================================================
 *          测试任务 (内部)
 * ================================================================ */

static TaskHandle_t ir_test_task_handle = NULL;

static void IR_TestTaskEntry__(void *argument)
{
    (void)argument;
    uint32_t last_send_time = 0;
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;

    IR_Init(ir_host_context.hcan, (uint8_t[]){IR_HOST_DEFAULT_MODULE_ID}, 1);
    vTaskDelay(200);

    tx_header.StdId = IR_HOST_CAN_ID_DATA(IR_HOST_DEFAULT_MODULE_ID);
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.TransmitGlobalTime = DISABLE;

    for (;;) {
        ir_debug.test_cycle++;
        ir_debug.task_state = (uint8_t)IR_GetTaskState();
        ir_debug.online_count = IR_GetOnlineCount();
        ir_debug.total_modules = IR_GetModuleCount();

        IR_Module_Node_t *mod1 = IR_FindModule__(IR_HOST_DEFAULT_MODULE_ID);
        if (mod1 != NULL) {
            ir_debug.mod1.discovered = mod1->discovered ? 1 : 0;
            ir_debug.mod1.online = mod1->online ? 1 : 0;
            ir_debug.mod1.busy = mod1->busy ? 1 : 0;
            ir_debug.mod1.consecutive_errors = mod1->consecutive_errors;
            ir_debug.mod1.total_rx_count = mod1->data_cache.total_rx_count;
            ir_debug.mod1.last_rx_time = mod1->last_rx_time;
            if (mod1->data_cache.valid) memcpy(ir_debug.rx_data, mod1->data_cache.raw_data, 8);
        }

        if (ir_debug.task_state == IR_HOST_TASK_STATE_RUNNING && ir_debug.online_count >= 1) {
            uint32_t now = xTaskGetTickCount();
            if (now - last_send_time >= 1000) {
                last_send_time = now;
                ir_debug.tx_data[0] = (uint8_t)(ir_debug.test_cycle & 0xFF);
                ir_debug.tx_data[1] = (uint8_t)((ir_debug.test_cycle >> 8) & 0xFF);
                ir_debug.tx_data[2]++;
                ir_debug.tx_data[3] = 0xDD;
                ir_debug.tx_data[4] = 0xEE;
                ir_debug.tx_data[5] = 0xFF;
                ir_debug.tx_data[6] = 0x11;
                ir_debug.tx_data[7] = 0x22;
                tx_header.DLC = 8;
                if (HAL_CAN_AddTxMessage(ir_host_context.hcan, &tx_header, ir_debug.tx_data, &tx_mailbox) == HAL_OK) {
                    ir_debug.stats.tx_success_count++;
                    ir_debug.data_changed_flag = 1;
                    ir_debug.update_timestamp = xTaskGetTickCount();
                } else {
                    ir_debug.stats.tx_fail_count++;
                    ir_debug.data_changed_flag = 0;
                }
            }
        }

        if (ir_debug.test_cycle % 100 == 0 && ir_debug.task_state == IR_HOST_TASK_STATE_DISCOVERY) {
            IR_ForceRediscover();
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void IR_StartTest(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL) return;
    ir_host_context.hcan = hcan;
    if (ir_test_task_handle == NULL) {
        xTaskCreate(IR_TestTaskEntry__, "IR_Test", 512, NULL, 4, &ir_test_task_handle);
    }
}
