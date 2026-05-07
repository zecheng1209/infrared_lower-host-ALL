#ifndef __INFRARED_HOST_H
#define __INFRARED_HOST_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdbool.h>
#include <string.h>

// CAN通信参数（经由CAN-IR桥接与红外从机通信）
#define IR_HOST_CAN_TIMEOUT_MS     500      // CAN发送超时（ms），应大于从机的IR_ACK_TIMEOUT_MS(100)
#define IR_HOST_MAX_RETRY_COUNT     3       // 最大重传次数，与从机IR_MAX_RETRY_COUNT一致
#define IR_HOST_FRAME_INTERVAL_MS   200     // CAN帧最小发送间隔（ms），原50ms太快导致下位机ACK发不出

// 与从机infrared.h保持一致的协议常量
#define IR_ACK_MAGIC                0xA5    // ACK帧魔术字节（与从机IR_ACK_MAGIC一致）
#define IR_NACK_MAGIC               0x5A    // NACK帧魔术字节（与从机IR_NACK_MAGIC一致）
#define IR_HOST_DEFAULT_MODULE_ID   0x10    // 默认模块ID，需与从机IR_MODULE_ID一致

// CAN ID 编码方案（方案C：ID中嵌入模块号）
// 11位标准CAN ID: [10:8]=帧类型(1=CMD/2=DATA/3=ACK), [7:0]=模块ID
// 示例: module_id=0x10 → CMD=0x110, DATA=0x210, ACK=0x310
#define IR_HOST_CAN_TYPE_CMD   1
#define IR_HOST_CAN_TYPE_DATA  2
#define IR_HOST_CAN_TYPE_ACK   3

#define IR_HOST_CAN_ID_BUILD(type, module_id)  (((uint32_t)(type) << 8) | (uint32_t)(module_id))
#define IR_HOST_CAN_ID_GET_TYPE(can_id)        ((uint8_t)((can_id) >> 8))
#define IR_HOST_CAN_ID_GET_MODULE(can_id)      ((uint8_t)((can_id) & 0xFF))

#define IR_HOST_IS_CMD_FRAME(can_id)   (IR_HOST_CAN_ID_GET_TYPE(can_id) == IR_HOST_CAN_TYPE_CMD)
#define IR_HOST_IS_DATA_FRAME(can_id)  (IR_HOST_CAN_ID_GET_TYPE(can_id) == IR_HOST_CAN_TYPE_DATA)
#define IR_HOST_IS_ACK_FRAME(can_id)   (IR_HOST_CAN_ID_GET_TYPE(can_id) == IR_HOST_CAN_TYPE_ACK)

// 兼容旧接口（单模块时可直接使用）
#define IR_HOST_CAN_ID_COMMAND(module_id)  IR_HOST_CAN_ID_BUILD(IR_HOST_CAN_TYPE_CMD, module_id)
#define IR_HOST_CAN_ID_DATA(module_id)     IR_HOST_CAN_ID_BUILD(IR_HOST_CAN_TYPE_DATA, module_id)
#define IR_HOST_CAN_ID_ACK(module_id)      IR_HOST_CAN_ID_BUILD(IR_HOST_CAN_TYPE_ACK, module_id)

#define IR_HOST_RX_QUEUE_SIZE       20
#define IR_HOST_MAX_MODULES         8

#define IR_HOST_POLL_INTERVAL_MS    300     // 轮询间隔（ms），原100ms太快
#define IR_HOST_ONLINE_TIMEOUT_MS   3000
#define IR_HOST_DATA_STALE_MS       500
#define IR_HOST_MAX_CRC_ERRORS      5
#define IR_HOST_CONSISTENT_COUNT    3

typedef enum {
    IR_HOST_CMD_PING = 0x01,
    IR_HOST_CMD_SEND_DATA = 0x02,
    IR_HOST_CMD_READ_STATUS = 0x03,
    IR_HOST_CMD_RESET = 0x04,
    IR_HOST_CMD_READ_SENSOR = 0x05,
    IR_HOST_CMD_READ_ALL = 0x06
} IR_Host_Command_t;//主机命令

typedef enum {
    IR_HOST_STATUS_IDLE = 0x00,
    IR_HOST_STATUS_SENDING = 0x01,
    IR_HOST_STATUS_WAIT_ACK = 0x02,
    IR_HOST_STATUS_SUCCESS = 0x03,
    IR_HOST_STATUS_TIMEOUT = 0x04,
    IR_HOST_STATUS_NACK = 0x05,
    IR_HOST_STATUS_ERROR = 0x06
// 主机响应帧结构体
} IR_Host_Status_t;//主机状态

typedef enum {
    IR_HOST_FRAME_TYPE_COMMAND = 0x01,
    IR_HOST_FRAME_TYPE_DATA = 0x02,
    IR_HOST_FRAME_TYPE_ACK = 0x03,
    IR_HOST_FRAME_TYPE_UNKNOWN = 0xFF
} IR_Host_FrameType_t;//主机帧类型

typedef enum {
    IR_HOST_TASK_STATE_INIT = 0,
    IR_HOST_TASK_STATE_DISCOVERY,
    IR_HOST_TASK_STATE_RUNNING,
    IR_HOST_TASK_STATE_ERROR
} IR_Host_TaskState_t;//主机任务状态

typedef enum {
    IR_DATA_CHECK_OK = 0,
    IR_DATA_CHECK_STALE,
    IR_DATA_CHECK_CRC_ERR,
    IR_DATA_CHECK_INCONSISTENT,
    IR_DATA_CHECK_OFFLINE,
    IR_DATA_CHECK_NO_MODULE
} IR_Data_CheckResult_t;//数据检查结果

typedef struct {
    uint32_t can_id;
    uint8_t module_id;
    uint8_t data[8];
    uint8_t dlc;
    uint32_t timestamp;
} IR_Host_RxFrame_t;//主机接收帧

typedef struct {
    uint8_t module_id;
    IR_Host_Status_t status;
    // 模块节点链表头指针
    uint8_t count;
    SemaphoreHandle_t mutex;
} IR_Module_List_t;//模块节点链表
    bool valid;
} IR_Host_ResponseFrame_t;

typedef struct {
    uint8_t raw_data[8];
    uint32_t update_timestamp;
    bool valid;
    uint8_t consistent_count;
    uint8_t crc_error_count;
    uint32_t total_rx_count;
    uint32_t total_crc_error_count;
} IR_Module_DataCache_t;//模块数据缓存

struct IR_Module_Node;

typedef struct IR_Module_Node {
    uint8_t module_id;
    IR_Host_Status_t status;
    IR_Host_ResponseFrame_t last_response;
    IR_Module_DataCache_t data_cache;
    uint32_t last_rx_time;
    uint32_t last_tx_time;
    bool online;
    bool busy;
    bool discovered;
    uint8_t poll_fail_count;
    uint8_t consecutive_errors;
    struct IR_Module_Node *next;
} IR_Module_Node_t;

typedef struct {
    IR_Module_Node_t *head;
    uint8_t count;
    SemaphoreHandle_t mutex;
} IR_Module_List_t;

typedef struct {
    QueueHandle_t rx_queue;
    IR_Module_List_t module_list;
    bool initialized;
    IR_Host_TaskState_t task_state;
    uint8_t current_poll_index;
    uint32_t last_poll_time;
    uint32_t discovery_start_time;
} IR_Host_Context_t;

extern IR_Host_Context_t ir_host_context;

void IR_Host_Init(void);
void IR_Host_StartTask(void);

bool IR_Host_AddModule(uint8_t module_id);
bool IR_Host_RemoveModule(uint8_t module_id);
IR_Module_Node_t* IR_Host_FindModule(uint8_t module_id);
IR_Module_Node_t* IR_Host_GetModuleByIndex(uint8_t index);
uint8_t IR_Host_GetModuleCount(void);
bool IR_Host_IsModuleOnline(uint8_t module_id);

bool IR_Host_SendCommand(uint8_t module_id, IR_Host_Command_t cmd, uint8_t *data, uint8_t length);
bool IR_Host_SendDataWithRetry(uint8_t module_id, uint8_t *data, uint8_t length, uint8_t max_retry);
bool IR_Host_Ping(uint8_t module_id, uint32_t timeout_ms);
bool IR_Host_ReadStatus(uint8_t module_id, uint8_t *status, uint32_t timeout_ms);
bool IR_Host_ResetModule(uint8_t module_id, uint32_t timeout_ms);

IR_Data_CheckResult_t IR_Host_CheckDataConsistency(uint8_t module_id);
IR_Data_CheckResult_t IR_Host_CheckAllModules(void);
bool IR_Host_GetModuleData(uint8_t module_id, uint8_t *data, uint8_t *length);
uint8_t IR_Host_GetOnlineCount(void);
IR_Host_TaskState_t IR_Host_GetTaskState(void);
void IR_Host_ForceRediscover(void);

void IR_Host_Receive_DataFrame_Ocan(uint32_t can_id, uint8_t *data, uint8_t dlc);
void IR_Host_Handle(uint8_t module_id, CAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint8_t dlc);
void IR_Host_ProcessRxFrame(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);
void IR_Host_Task(void *argument);

uint8_t IR_Host_CRC8(uint8_t *data, uint8_t length);

void IR_Host_ConfigCanFilter(void);
void IR_Host_StartCan(void);
void IR_Host_TxMailboxCompleteCallback(CAN_HandleTypeDef *hcan);

void IR_Test_Task(void *argument);
void IR_Test_StartTask(void);

#endif
