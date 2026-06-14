#include "infrared_host_lite.h"
#include "can.h"

IR_Host_Context_t ir_host_context = {0};
IR_Debug_Data_t   ir_debug = {0};

static TaskHandle_t ir_host_task_handle = NULL;

/* ======================== 内部函数声明 ======================== */

static void IR_ProcessRxQueue__(void);
static void IR_HandleAckNack__(uint8_t module_id, uint32_t id, uint8_t *data, uint8_t dlc);
static void IR_EnqueueRxFrame__(uint32_t can_id, uint8_t *data, uint8_t dlc);
static void IR_DiscoveryPhase__(void);
static void IR_PollModuleById__(uint8_t module_id);
static void IR_UpdateOnlineStatus__(void);
static void IR_TaskEntry__(void *argument);
static IR_Module_Node_t* IR_FindModule__(uint8_t module_id);
static void IR_ReleaseModule__(IR_Module_Node_t *module);
static IR_Module_Node_t* IR_FindModuleByIndexNoLock__(uint8_t index);
static void IR_PerformVote__(void);
static bool IR_SendCommand__(uint8_t module_id, IR_Host_Command_t cmd, uint8_t *data, uint8_t length);

/* ================================================================
 *          初始化
 * ================================================================ */

bool IR_Init(CAN_HandleTypeDef *hcan, uint8_t *module_ids, uint8_t count)
{
    if (hcan == NULL || module_ids == NULL || count == 0 || count > IR_HOST_MAX_MODULES)
        return false;

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

    for (uint8_t i = 0; i < count; i++) {
        IR_AddModule(module_ids[i]);
    }

    if (ir_host_task_handle == NULL) {
        xTaskCreate(IR_TaskEntry__, "IRHost", 1024, NULL, 3, &ir_host_task_handle);
    }

    return true;
}

/* ================================================================
 *          阻塞发送
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
    if (length > 8 || length == 0) { IR_ReleaseModule__(module); return false; }

    uint32_t timeout_ms = ir_host_context.can_timeout_ms;

    for (uint8_t r = 0; r < retry; r++) {
        /* 等待上一轮 busy 被清除 */
        uint32_t busy_wait_start = xTaskGetTickCount();
        while (module->busy) {
            IR_ProcessRxQueue__();
            if ((xTaskGetTickCount() - busy_wait_start) > timeout_ms) {
                module->busy = false;
                module->status = IR_HOST_STATUS_TIMEOUT;
                break;
            }
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
                    if (module->status == IR_HOST_STATUS_SUCCESS) { IR_ReleaseModule__(module); return true; }
                    if (module->status == IR_HOST_STATUS_NACK) break;
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            if (module->busy) {
                module->busy = false;
                module->status = IR_HOST_STATUS_TIMEOUT;
            }
        } else {
            module->busy = false;
            module->status = IR_HOST_STATUS_ERROR;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }

    IR_ReleaseModule__(module);
    return false;
}

/* ================================================================
 *          非阻塞发送
 * ================================================================ */

bool IR_SendAsync(uint8_t module_id, uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];

    if (ir_host_context.hcan == NULL) return false;

    IR_Module_Node_t *module = IR_FindModule__(module_id);
    if (module == NULL) return false;
    if (length > 8 || length == 0) { IR_ReleaseModule__(module); return false; }

    /* busy超时保护：如果下位机丢弃了RELIABLE_DATA帧导致无ACK/NACK回应，
     * busy会一直卡住。超过1秒强制释放，允许重试 */
    if (module->busy) {
        uint32_t busy_elapsed = xTaskGetTickCount() - module->last_tx_time;
        if (busy_elapsed < pdMS_TO_TICKS(1000)) { IR_ReleaseModule__(module); return false; }
        module->busy = false;
        module->status = IR_HOST_STATUS_TIMEOUT;
    }

    if ((xTaskGetTickCount() - module->last_tx_time) < IR_HOST_FRAME_INTERVAL_MS) { IR_ReleaseModule__(module); return false; }

    memset(tx_data, 0, 8);
    memcpy(tx_data, data, length);

    tx_header.StdId = IR_HOST_CAN_ID_RELIABLE_DATA(module_id);
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
        IR_ReleaseModule__(module);
        return false;
    }

    module->last_tx_time = xTaskGetTickCount();
    module->status = IR_HOST_STATUS_WAIT_ACK;
    IR_ReleaseModule__(module);
    return true;
}

/* ================================================================
 *          数据读取
 * ================================================================ */

bool IR_Read(uint8_t module_id, uint8_t *data, uint8_t *length)
{
    IR_Module_Node_t *module = IR_FindModule__(module_id);
    if (module == NULL) return false;
    if (!module->data_cache.valid) { IR_ReleaseModule__(module); return false; }

    if (data != NULL) memcpy(data, module->data_cache.raw_data, 8);
    if (length != NULL) *length = module->last_response.length > 8 ? 8 : module->last_response.length;
    IR_ReleaseModule__(module);
    return true;
}

bool IR_IsOnline(uint8_t module_id)
{
    IR_Module_Node_t *module = IR_FindModule__(module_id);
    if (module == NULL) return false;
    bool online = module->online;
    IR_ReleaseModule__(module);
    return online;
}

/* ================================================================
 *          CAN中断回调
 * ================================================================ */

void IR_OnCanRx(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data)
{
    if (rx_header == NULL || rx_data == NULL) return;

    uint8_t dlc = rx_header->DLC > 8 ? 8 : rx_header->DLC;

    /* debug: 统计与最近帧 */
    ir_debug.all_rx_frame_count++;
    ir_debug.last_rx_can_id = rx_header->StdId;
    ir_debug.last_rx_dlc = dlc;
    memcpy(ir_debug.last_rx_data, rx_data, dlc);
    ir_debug.rx_data_len = dlc;

    /* debug: 历史记录 */
    IR_Debug_RxHistory_t *h = &ir_debug.rx_history[ir_debug.history_write_idx];
    h->can_id = rx_header->StdId;
    h->dlc = dlc;
    memcpy(h->data, rx_data, dlc);
    ir_debug.history_write_idx = (ir_debug.history_write_idx + 1) % IR_DEBUG_RX_HISTORY_SIZE;
    if (ir_debug.history_count < IR_DEBUG_RX_HISTORY_SIZE) ir_debug.history_count++;

    if (IR_HOST_IS_ACK_FRAME(rx_header->StdId)) {
        ir_debug.ack_frame_count++;
    } else if (IR_HOST_IS_DATA_FRAME(rx_header->StdId) || IR_HOST_IS_RELIABLE_DATA_FRAME(rx_header->StdId)) {
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
    if (!module->online || !module->discovered) { IR_ReleaseModule__(module); return IR_DATA_CHECK_OFFLINE; }
    if (!module->data_cache.valid) { IR_ReleaseModule__(module); return IR_DATA_CHECK_CRC_ERR; }
    if ((xTaskGetTickCount() - module->data_cache.update_timestamp) > IR_HOST_DATA_STALE_MS)
        { IR_ReleaseModule__(module); return IR_DATA_CHECK_STALE; }
    IR_ReleaseModule__(module);
    return IR_DATA_CHECK_OK;
}

bool IR_AddModule(uint8_t module_id)
{
    if (!ir_host_context.initialized) return false;
    if (ir_host_context.module_list.count >= IR_HOST_MAX_MODULES) return false;

    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);

    IR_Module_Node_t *cur = ir_host_context.module_list.head;
    while (cur != NULL) {
        if (cur->module_id == module_id) {
            xSemaphoreGive(ir_host_context.module_list.mutex);
            return false;
        }
        cur = cur->next;
    }

    IR_Module_Node_t *node = (IR_Module_Node_t *)pvPortMalloc(sizeof(IR_Module_Node_t));
    if (node == NULL) {
        xSemaphoreGive(ir_host_context.module_list.mutex);
        return false;
    }

    memset(node, 0, sizeof(IR_Module_Node_t));
    node->module_id = module_id;
    node->status = IR_HOST_STATUS_IDLE;
    node->last_tx_time = xTaskGetTickCount() - IR_HOST_FRAME_INTERVAL_MS - 1;
    node->ref_count = 1;      // 链表持有的引用
    node->deleted = false;
    node->next = ir_host_context.module_list.head;
    ir_host_context.module_list.head = node;
    ir_host_context.module_list.count++;

    xSemaphoreGive(ir_host_context.module_list.mutex);
    return true;
}

bool IR_RemoveModule(uint8_t module_id)
{
    if (!ir_host_context.initialized) return false;

    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);

    IR_Module_Node_t **cur = &ir_host_context.module_list.head;
    while (*cur != NULL) {
        if ((*cur)->module_id == module_id && !(*cur)->deleted) {
            IR_Module_Node_t *rm = *cur;
            *cur = (*cur)->next;
            ir_host_context.module_list.count--;

            // ★ 标记删除，释放链表引用，若外部无引用则立即释放
            rm->deleted = true;
            if (rm->ref_count > 0) {
                rm->ref_count--;  // 释放链表持有的引用
            }
            bool should_free = (rm->ref_count == 0);

            xSemaphoreGive(ir_host_context.module_list.mutex);

            if (should_free) {
                vPortFree(rm);
            }
            return true;
        }
        cur = &(*cur)->next;
    }

    xSemaphoreGive(ir_host_context.module_list.mutex);
    return false;
}

uint8_t IR_GetOnlineCount(void)
{
    uint8_t count = 0;
    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
    IR_Module_Node_t *cur = ir_host_context.module_list.head;
    while (cur != NULL) {
        if (cur->online && cur->discovered) count++;
        cur = cur->next;
    }
    xSemaphoreGive(ir_host_context.module_list.mutex);
    return count;
}

void IR_ForceRediscover(void)
{
    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
    IR_Module_Node_t *cur = ir_host_context.module_list.head;
    while (cur != NULL) {
        cur->discovered = false;
        cur->online = false;
        cur->poll_fail_count = 0;
        cur->consecutive_errors = 0;
        memset(&cur->data_cache, 0, sizeof(IR_Module_DataCache_t));
        cur = cur->next;
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

/* ================================================================
 *          内部: 命令发送（非阻塞，仅组帧+发送）
 * ================================================================ */

static bool IR_SendCommand__(uint8_t module_id, IR_Host_Command_t cmd, uint8_t *data, uint8_t length)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    uint8_t tx_data[8];

    if (ir_host_context.hcan == NULL) return false;

    IR_Module_Node_t *module = IR_FindModule__(module_id);
    if (module == NULL) return false;
    if (module->busy || length > 6) { IR_ReleaseModule__(module); return false; }

    if ((xTaskGetTickCount() - module->last_tx_time) < IR_HOST_FRAME_INTERVAL_MS) { IR_ReleaseModule__(module); return false; }

    memset(tx_data, 0, 8);
    tx_data[0] = module_id;
    tx_data[1] = cmd;
    if (data != NULL && length > 0) memcpy(&tx_data[2], data, length);

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
        IR_ReleaseModule__(module);
        return false;
    }

    module->last_tx_time = xTaskGetTickCount();
    module->status = IR_HOST_STATUS_WAIT_ACK;
    IR_ReleaseModule__(module);
    return true;
}

/* ================================================================
 *          Ping / Reset（带超时阻塞等待）
 * ================================================================ */

bool IR_PingTimeout(uint8_t module_id, uint32_t timeout_ms)
{
    IR_Module_Node_t *module = IR_FindModule__(module_id);
    if (module == NULL) return false;
    if (module->busy) { IR_ReleaseModule__(module); return false; }
    if ((xTaskGetTickCount() - module->last_tx_time) < IR_HOST_FRAME_INTERVAL_MS) { IR_ReleaseModule__(module); return false; }

    if (!IR_SendCommand__(module_id, IR_HOST_CMD_PING, NULL, 0)) { IR_ReleaseModule__(module); return false; }

    uint32_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < timeout_ms) {
        IR_ProcessRxQueue__();
        if (!module->busy) {
            bool success = (module->status == IR_HOST_STATUS_SUCCESS);
            IR_ReleaseModule__(module);
            return success;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    module->busy = false;
    module->status = IR_HOST_STATUS_TIMEOUT;
    IR_ReleaseModule__(module);
    return false;
}

bool IR_ResetTimeout(uint8_t module_id, uint32_t timeout_ms)
{
    IR_Module_Node_t *module = IR_FindModule__(module_id);
    if (module == NULL) return false;
    if (module->busy) { IR_ReleaseModule__(module); return false; }
    if ((xTaskGetTickCount() - module->last_tx_time) < IR_HOST_FRAME_INTERVAL_MS) { IR_ReleaseModule__(module); return false; }

    if (!IR_SendCommand__(module_id, IR_HOST_CMD_RESET, NULL, 0)) { IR_ReleaseModule__(module); return false; }

    uint32_t start = xTaskGetTickCount();
    while ((xTaskGetTickCount() - start) < timeout_ms) {
        IR_ProcessRxQueue__();
        if (!module->busy) { IR_ReleaseModule__(module); return true; }
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    module->busy = false;
    module->status = IR_HOST_STATUS_TIMEOUT;
    IR_ReleaseModule__(module);
    return false;
}

/* ================================================================
 *          内部: 模块查找
 * ================================================================ */

static IR_Module_Node_t* IR_FindModule__(uint8_t module_id)
{
    if (!ir_host_context.initialized) return NULL;

    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
    IR_Module_Node_t *cur = ir_host_context.module_list.head;
    while (cur != NULL) {
        if (cur->module_id == module_id && !cur->deleted) {
            cur->ref_count++;  // ★ 引用计数+1，调用方用完必须 IR_ReleaseModule__
            xSemaphoreGive(ir_host_context.module_list.mutex);
            return cur;
        }
        cur = cur->next;
    }
    xSemaphoreGive(ir_host_context.module_list.mutex);
    return NULL;
}

static void IR_ReleaseModule__(IR_Module_Node_t *module)
{
    if (module == NULL) return;

    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
    if (module->ref_count > 0) {
        module->ref_count--;
    }
    bool should_free = (module->ref_count == 0 && module->deleted);
    xSemaphoreGive(ir_host_context.module_list.mutex);

    if (should_free) {
        vPortFree(module);
    }
}

static IR_Module_Node_t* IR_FindModuleByIndexNoLock__(uint8_t index)
{
    IR_Module_Node_t *m = ir_host_context.module_list.head;
    for (uint8_t i = 0; i < index && m != NULL; i++)
        m = m->next;
    return m;
}

/* ================================================================
 *          内部: ACK/NACK 处理
 * ================================================================ */

static void IR_HandleAckNack__(uint8_t module_id, uint32_t id, uint8_t *data, uint8_t dlc)
{
    if (!IR_HOST_IS_ACK_FRAME(id)) return;

    IR_Module_Node_t *module = IR_FindModule__(module_id);
    if (module == NULL) return;

    if (dlc >= 2 && data[0] == IR_ACK_MAGIC && data[1] == IR_ACK_MAGIC) {
        module->status = IR_HOST_STATUS_SUCCESS;
        module->busy = false;
        module->consecutive_errors = 0;
        module->online = true;
        module->discovered = true;
        module->last_rx_time = xTaskGetTickCount();
    } else if (dlc >= 2 && data[0] == IR_NACK_MAGIC && data[1] == IR_NACK_MAGIC) {
        module->status = IR_HOST_STATUS_NACK;
        module->busy = false;
    }
    IR_ReleaseModule__(module);
}

/* ================================================================
 *          内部: 接收队列处理（统一入口，消除重复代码）
 * ================================================================ */

static void IR_ProcessRxQueue__(void)
{
    IR_Host_RxFrame_t rx_frame;
    while (xQueueReceive(ir_host_context.rx_queue, &rx_frame, 0) == pdTRUE) {
        if (IR_HOST_IS_ACK_FRAME(rx_frame.can_id)) {
            IR_HandleAckNack__(rx_frame.module_id, rx_frame.can_id, rx_frame.data, rx_frame.dlc);
        } else if (IR_HOST_IS_DATA_FRAME(rx_frame.can_id) || IR_HOST_IS_RELIABLE_DATA_FRAME(rx_frame.can_id)) {
            IR_Module_Node_t *module = IR_FindModule__(rx_frame.module_id);
            if (module != NULL && rx_frame.dlc >= 1) {
                uint8_t len = rx_frame.dlc > 8 ? 8 : rx_frame.dlc;
                module->data_cache.total_rx_count++;
                memcpy(module->data_cache.raw_data, rx_frame.data, len);
                module->data_cache.update_timestamp = rx_frame.timestamp;
                module->data_cache.valid = true;
                module->last_response.module_id = rx_frame.module_id;
                module->last_response.status = IR_HOST_STATUS_SUCCESS;
                module->last_response.length = len;
                module->last_response.timestamp = rx_frame.timestamp;
                module->last_response.valid = true;
                memcpy(module->last_response.data, rx_frame.data, len);
                module->last_rx_time = rx_frame.timestamp;
                module->online = true;
                module->busy = false;
                module->discovered = true;
                module->poll_fail_count = 0;
                module->consecutive_errors = 0;
                IR_ReleaseModule__(module);
            }
        }
    }
}

/* ================================================================
 *          内部: 发现阶段
 * ================================================================ */

static void IR_DiscoveryPhase__(void)
{
    static uint8_t discover_index = 0;
    uint32_t now = xTaskGetTickCount();

    if (now - ir_host_context.discovery_start_time > 5000) {
        ir_host_context.task_state = IR_HOST_TASK_STATE_RUNNING;
        discover_index = 0;
        return;
    }

    uint8_t target_id = 0;
    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
    {
        IR_Module_Node_t *m = IR_FindModuleByIndexNoLock__(discover_index);
        if (m != NULL && !m->discovered && !m->busy &&
            (now - m->last_tx_time) > ir_host_context.poll_interval_ms) {
            target_id = m->module_id;
        }
    }
    xSemaphoreGive(ir_host_context.module_list.mutex);

    if (target_id != 0) {
        if (IR_PingTimeout(target_id, 200)) {
            IR_Module_Node_t *module = IR_FindModule__(target_id);
            if (module != NULL) {
                module->discovered = true;
                module->online = true;
                module->last_rx_time = xTaskGetTickCount();
                IR_ReleaseModule__(module);
            }
        } else {
            IR_Module_Node_t *module = IR_FindModule__(target_id);
            if (module != NULL) {
                module->poll_fail_count++;
                IR_ReleaseModule__(module);
            }
        }
    }

    discover_index++;
    if (discover_index >= ir_host_context.module_list.count) discover_index = 0;
    vTaskDelay(pdMS_TO_TICKS(50));
}

/* ================================================================
 *          内部: 轮询单个模块
 * ================================================================ */

static void IR_PollModuleById__(uint8_t module_id)
{
    uint32_t now = xTaskGetTickCount();
    IR_Module_Node_t *module = IR_FindModule__(module_id);
    if (module == NULL) return;

    /* busy超时保护 */
    if (module->busy && !module->online &&
        (now - module->last_tx_time) > ir_host_context.can_timeout_ms) {
        module->busy = false;
        module->status = IR_HOST_STATUS_TIMEOUT;
    }
    if (module->busy) { IR_ReleaseModule__(module); return; }

    if (!module->online || !module->discovered) {
        if (now - module->last_tx_time > ir_host_context.poll_interval_ms * 2) {
            IR_ReleaseModule__(module);  // ★ 释放原引用，后续重新获取
            if (IR_PingTimeout(module_id, 300)) {
                module = IR_FindModule__(module_id);
                if (module != NULL) {
                    module->discovered = true;
                    module->online = true;
                    module->last_rx_time = xTaskGetTickCount();
                    IR_ReleaseModule__(module);
                }
            } else {
                module = IR_FindModule__(module_id);
                if (module != NULL) {
                    module->poll_fail_count++;
                    module->consecutive_errors++;
                    if (module->consecutive_errors > 10) {
                        module->online = false;
                        module->discovered = false;
                        module->data_cache.valid = false;
                    }
                    IR_ReleaseModule__(module);
                }
            }
        } else {
            IR_ReleaseModule__(module);
        }
    } else {
        if (now - module->last_rx_time < ir_host_context.poll_interval_ms * 2) {
            IR_ReleaseModule__(module);
            return;
        }
        if (now - module->last_tx_time >= ir_host_context.poll_interval_ms) {
            IR_ReleaseModule__(module);  // ★ 释放原引用，后续重新获取
            if (IR_PingTimeout(module_id, 300)) {
                module = IR_FindModule__(module_id);
                if (module != NULL) {
                    module->consecutive_errors = 0;
                    module->last_rx_time = xTaskGetTickCount();
                    IR_ReleaseModule__(module);
                }
            } else {
                module = IR_FindModule__(module_id);
                if (module != NULL) {
                    module->poll_fail_count++;
                    module->consecutive_errors++;
                    if (module->consecutive_errors > 5) {
                        module->online = false;
                        module->data_cache.valid = false;
                    }
                    IR_ReleaseModule__(module);
                }
            }
        } else {
            IR_ReleaseModule__(module);
        }
    }
}

/* ================================================================
 *          内部: 在线状态更新
 * ================================================================ */

static void IR_UpdateOnlineStatus__(void)
{
    uint32_t now = xTaskGetTickCount();

    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
    IR_Module_Node_t *cur = ir_host_context.module_list.head;
    while (cur != NULL) {
        if (cur->online && (now - cur->last_rx_time) > ir_host_context.online_timeout_ms) {
            cur->online = false;
            cur->data_cache.valid = false;
            cur->busy = false;
            cur->status = IR_HOST_STATUS_TIMEOUT;
        }
        if (cur->data_cache.valid && (now - cur->data_cache.update_timestamp) > IR_HOST_DATA_STALE_MS * 3) {
            cur->data_cache.valid = false;
        }
        cur = cur->next;
    }
    xSemaphoreGive(ir_host_context.module_list.mutex);
}

/* ================================================================
 *          内部: 投票
 * ================================================================ */

static void IR_PerformVote__(void)
{
    IR_Module_Node_t *modules[IR_HOST_MAX_MODULES];
    uint8_t n = 0;
    uint32_t window_start = ir_host_context.vote_window_start_time;

    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
    IR_Module_Node_t *cur = ir_host_context.module_list.head;
    while (cur != NULL) {
        if (cur->data_cache.valid &&
            cur->data_cache.update_timestamp >= window_start) {
            if (n < IR_HOST_MAX_MODULES) modules[n++] = cur;
        }
        cur = cur->next;
    }
    xSemaphoreGive(ir_host_context.module_list.mutex);

    if (n == 0) return;

    if (n == 1) {
        memcpy(ir_host_context.vote_result.data, modules[0]->data_cache.raw_data, 8);
        ir_host_context.vote_result.dlc = modules[0]->last_response.length;
        ir_host_context.vote_result.valid = true;
        ir_host_context.vote_result.agree_count = 1;
        ir_host_context.vote_result.total_count = 1;
        ir_host_context.vote_result.timestamp = xTaskGetTickCount();
        ir_host_context.total_votes++;
    } else {
        uint8_t best_idx = 0, best_count = 0;
        for (uint8_t i = 0; i < n; i++) {
            uint8_t count = 1;
            for (uint8_t j = i + 1; j < n; j++) {
                if (modules[i]->last_response.length == modules[j]->last_response.length &&
                    memcmp(modules[i]->data_cache.raw_data, modules[j]->data_cache.raw_data,
                           modules[i]->last_response.length) == 0) {
                    count++;
                }
            }
            if (count > best_count) { best_count = count; best_idx = i; }
        }

        memcpy(ir_host_context.vote_result.data, modules[best_idx]->data_cache.raw_data, 8);
        ir_host_context.vote_result.dlc = modules[best_idx]->last_response.length;
        ir_host_context.vote_result.valid = true;
        ir_host_context.vote_result.agree_count = best_count;
        ir_host_context.vote_result.total_count = n;
        ir_host_context.vote_result.timestamp = xTaskGetTickCount();
        ir_host_context.total_votes++;
    }

    /* 更新debug */
    memcpy(ir_debug.voted_data, ir_host_context.vote_result.data, 8);
    ir_debug.voted_dlc = ir_host_context.vote_result.dlc;
    ir_debug.voted_agree_count = ir_host_context.vote_result.agree_count;
    ir_debug.voted_total_count = ir_host_context.vote_result.total_count;
    ir_debug.vote_count = ir_host_context.total_votes;
    ir_debug.voted_valid = ir_host_context.vote_result.valid ? 1 : 0;
}

bool IR_ReadVoted(uint8_t *data, uint8_t *length)
{
    if (!ir_host_context.vote_result.valid) return false;
    if (data != NULL) memcpy(data, ir_host_context.vote_result.data, 8);
    if (length != NULL) *length = ir_host_context.vote_result.dlc;
    return true;
}

IR_Vote_Result_t* IR_GetVoteResult(void)
{
    return &ir_host_context.vote_result;
}

/* ================================================================
 *          后台任务（统一使用 IR_ProcessRxQueue__ 处理接收）
 * ================================================================ */

static void IR_TaskEntry__(void *argument)
{
    (void)argument;

    ir_host_context.task_state = IR_HOST_TASK_STATE_DISCOVERY;
    ir_host_context.discovery_start_time = xTaskGetTickCount();

    for (;;) {
        /* 统一处理接收队列 */
        IR_ProcessRxQueue__();

        /* Bus-off 自动恢复 */
        if (ir_host_context.hcan != NULL) {
            if (ir_host_context.hcan->Instance->ESR & CAN_ESR_BOFF) {
                HAL_CAN_Stop(ir_host_context.hcan);
                HAL_CAN_Start(ir_host_context.hcan);
            }
        }

        /* 投票窗口检测 — ★ 加锁保护链表遍历 */
        if (!ir_host_context.vote_window_active) {
            xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
            {
                IR_Module_Node_t *cur = ir_host_context.module_list.head;
                while (cur != NULL) {
                    if (cur->data_cache.valid &&
                        cur->data_cache.update_timestamp > ir_host_context.vote_result.timestamp) {
                        ir_host_context.vote_window_active = true;
                        ir_host_context.vote_window_start_time = cur->data_cache.update_timestamp;
                        break;
                    }
                    cur = cur->next;
                }
            }
            xSemaphoreGive(ir_host_context.module_list.mutex);
        }

        if (ir_host_context.vote_window_active) {
            uint32_t elapsed = xTaskGetTickCount() - ir_host_context.vote_window_start_time;

            // ★ 加锁+内联计算 online_cnt 避免嵌套锁
            bool all_reported = false;
            uint8_t online_cnt = 0;
            uint8_t fresh = 0;
            xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
            {
                IR_Module_Node_t *cur = ir_host_context.module_list.head;
                while (cur != NULL) {
                    if (cur->online && cur->discovered) online_cnt++;
                    if (cur->online && cur->data_cache.valid &&
                        cur->data_cache.update_timestamp >= ir_host_context.vote_window_start_time) {
                        fresh++;
                    }
                    cur = cur->next;
                }
            }
            xSemaphoreGive(ir_host_context.module_list.mutex);

            if (online_cnt > 0) {
                all_reported = (fresh >= online_cnt);
            }

            if (all_reported || elapsed >= IR_HOST_VOTE_WINDOW_MS) {
                IR_PerformVote__();
                ir_host_context.vote_window_active = false;
            }
        }

        /* 更新debug状态 */
        ir_debug.task_state = (uint8_t)ir_host_context.task_state;
        ir_debug.online_count = IR_GetOnlineCount();
        ir_debug.total_modules = IR_GetModuleCount();

        /* 更新各模块debug (id + last_rx_time) — ★ 加锁保护链表遍历 */
        {
            xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
            {
                IR_Module_Node_t *mod = ir_host_context.module_list.head;
                ir_debug.mod_count = 0;
                while (mod != NULL && ir_debug.mod_count < IR_HOST_MAX_MODULES) {
                    ir_debug.mods[ir_debug.mod_count].module_id = mod->module_id;
                    ir_debug.mods[ir_debug.mod_count].last_rx_time = mod->last_rx_time;
                    ir_debug.mod_count++;
                    mod = mod->next;
                }
            }
            xSemaphoreGive(ir_host_context.module_list.mutex);
        }

        /* 任务状态机 */
        switch (ir_host_context.task_state) {
            case IR_HOST_TASK_STATE_DISCOVERY:
                IR_DiscoveryPhase__();
                break;

            case IR_HOST_TASK_STATE_RUNNING:
            {
                uint32_t now = xTaskGetTickCount();
                if (now - ir_host_context.last_poll_time >= ir_host_context.poll_interval_ms) {
                    ir_host_context.last_poll_time = now;
                    uint8_t poll_id = 0;
                    xSemaphoreTake(ir_host_context.module_list.mutex, portMAX_DELAY);
                    {
                        IR_Module_Node_t *m = IR_FindModuleByIndexNoLock__(ir_host_context.current_poll_index);
                        if (m != NULL) poll_id = m->module_id;
                    }
                    xSemaphoreGive(ir_host_context.module_list.mutex);
                    if (poll_id != 0) IR_PollModuleById__(poll_id);
                    ir_host_context.current_poll_index++;
                    if (ir_host_context.current_poll_index >= ir_host_context.module_list.count) {
                        ir_host_context.current_poll_index = 0;
                        IR_UpdateOnlineStatus__();
                    }
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
