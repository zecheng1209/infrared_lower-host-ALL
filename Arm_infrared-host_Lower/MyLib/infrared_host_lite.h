#ifndef __INFRARED_HOST_LITE_H
#define __INFRARED_HOST_LITE_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdbool.h>
#include <string.h>

/* ======================== 可调参数 ======================== */

#define IR_HOST_CAN_TIMEOUT_MS     500
#define IR_HOST_MAX_RETRY_COUNT    3
#define IR_HOST_FRAME_INTERVAL_MS  200

#define IR_ACK_MAGIC               0xA5
#define IR_NACK_MAGIC              0x5A

#define IR_HOST_DEFAULT_MODULE_ID  0x10
#define IR_HOST_RX_QUEUE_SIZE      20
#define IR_HOST_MAX_MODULES        8
#define IR_HOST_POLL_INTERVAL_MS   300
#define IR_HOST_ONLINE_TIMEOUT_MS  3000
#define IR_HOST_DATA_STALE_MS      500
#define IR_HOST_VOTE_WINDOW_MS     50

/* ======================== CAN ID 编码 ======================== */
/* [10:8]=帧类型, [7:0]=模块ID
 * module_id=0x10 → CMD=0x110, DATA=0x210, ACK=0x310, RDATA=0x410 */

#define IR_HOST_CAN_TYPE_CMD           1
#define IR_HOST_CAN_TYPE_DATA          2
#define IR_HOST_CAN_TYPE_ACK           3
#define IR_HOST_CAN_TYPE_RELIABLE_DATA 4

#define IR_HOST_CAN_ID_BUILD(type, mid)     (((uint32_t)(type) << 8) | (uint32_t)(mid))
#define IR_HOST_CAN_ID_GET_TYPE(can_id)     ((uint8_t)((can_id) >> 8))
#define IR_HOST_CAN_ID_GET_MODULE(can_id)   ((uint8_t)((can_id) & 0xFF))

#define IR_HOST_IS_ACK_FRAME(can_id)           (IR_HOST_CAN_ID_GET_TYPE(can_id) == IR_HOST_CAN_TYPE_ACK)
#define IR_HOST_IS_DATA_FRAME(can_id)          (IR_HOST_CAN_ID_GET_TYPE(can_id) == IR_HOST_CAN_TYPE_DATA)
#define IR_HOST_IS_RELIABLE_DATA_FRAME(can_id) (IR_HOST_CAN_ID_GET_TYPE(can_id) == IR_HOST_CAN_TYPE_RELIABLE_DATA)

#define IR_HOST_CAN_ID_COMMAND(mid)        IR_HOST_CAN_ID_BUILD(IR_HOST_CAN_TYPE_CMD, mid)
#define IR_HOST_CAN_ID_DATA(mid)           IR_HOST_CAN_ID_BUILD(IR_HOST_CAN_TYPE_DATA, mid)
#define IR_HOST_CAN_ID_ACK(mid)            IR_HOST_CAN_ID_BUILD(IR_HOST_CAN_TYPE_ACK, mid)
#define IR_HOST_CAN_ID_RELIABLE_DATA(mid)  IR_HOST_CAN_ID_BUILD(IR_HOST_CAN_TYPE_RELIABLE_DATA, mid)

/* ======================== 枚举 ======================== */

typedef enum {
    IR_HOST_CMD_PING = 0x01,
    IR_HOST_CMD_SEND_DATA = 0x02,
    IR_HOST_CMD_READ_STATUS = 0x03,
    IR_HOST_CMD_RESET = 0x04,
} IR_Host_Command_t;

typedef enum {
    IR_HOST_STATUS_IDLE = 0x00,
    IR_HOST_STATUS_SENDING = 0x01,
    IR_HOST_STATUS_WAIT_ACK = 0x02,
    IR_HOST_STATUS_SUCCESS = 0x03,
    IR_HOST_STATUS_TIMEOUT = 0x04,
    IR_HOST_STATUS_NACK = 0x05,
    IR_HOST_STATUS_ERROR = 0x06
} IR_Host_Status_t;

typedef enum {
    IR_HOST_TASK_STATE_INIT = 0,
    IR_HOST_TASK_STATE_DISCOVERY,
    IR_HOST_TASK_STATE_RUNNING,
    IR_HOST_TASK_STATE_ERROR
} IR_Host_TaskState_t;

typedef enum {
    IR_DATA_CHECK_OK = 0,
    IR_DATA_CHECK_STALE,
    IR_DATA_CHECK_CRC_ERR,
    IR_DATA_CHECK_OFFLINE,
    IR_DATA_CHECK_NO_MODULE
} IR_Data_CheckResult_t;

/* ======================== 投票结果 ======================== */

typedef struct {
    uint8_t  data[8];
    uint8_t  dlc;
    bool     valid;
    uint8_t  agree_count;
    uint8_t  total_count;
    uint32_t timestamp;
} IR_Vote_Result_t;

/* ======================== 内部类型 ======================== */

typedef struct {
    uint32_t can_id;
    uint8_t  module_id;
    uint8_t  data[8];
    uint8_t  dlc;
    uint32_t timestamp;
} IR_Host_RxFrame_t;

typedef struct {
    uint8_t           module_id;
    IR_Host_Status_t  status;
    bool              valid;
    uint8_t           length;
    uint8_t           data[8];
    uint32_t          timestamp;
} IR_Host_ResponseFrame_t;

typedef struct {
    uint8_t  raw_data[8];
    uint32_t update_timestamp;
    bool     valid;
    uint32_t total_rx_count;
} IR_Module_DataCache_t;

typedef struct IR_Module_Node {
    uint8_t               module_id;
    IR_Host_Status_t      status;
    IR_Host_ResponseFrame_t last_response;
    IR_Module_DataCache_t data_cache;
    uint32_t              last_rx_time;
    uint32_t              last_tx_time;
    bool                  online;
    bool                  busy;
    bool                  discovered;
    uint8_t               poll_fail_count;
    uint8_t               consecutive_errors;
    struct IR_Module_Node *next;
    volatile uint8_t      ref_count;   ///< 引用计数
    bool                  deleted;     ///< 标记删除
} IR_Module_Node_t;

typedef struct {
    IR_Module_Node_t *head;
    uint8_t           count;
    SemaphoreHandle_t mutex;
} IR_Module_List_t;

typedef struct {
    QueueHandle_t       rx_queue;
    IR_Module_List_t    module_list;
    bool                initialized;
    IR_Host_TaskState_t task_state;
    uint8_t             current_poll_index;
    uint32_t            last_poll_time;
    uint32_t            discovery_start_time;
    CAN_HandleTypeDef  *hcan;
    uint32_t            can_timeout_ms;
    uint8_t             max_retry;
    uint32_t            poll_interval_ms;
    uint32_t            online_timeout_ms;
    /* 投票 */
    bool                vote_window_active;
    uint32_t            vote_window_start_time;
    IR_Vote_Result_t    vote_result;
    uint32_t            total_votes;
} IR_Host_Context_t;

/* ======================== Debug 结构体（精简） ======================== */

#define IR_DEBUG_RX_HISTORY_SIZE 4

typedef struct {
    uint32_t can_id;
    uint8_t  dlc;
    uint8_t  data[8];
} IR_Debug_RxHistory_t;

typedef struct {
    /* 接收统计 */
    uint32_t all_rx_frame_count;
    uint32_t ack_frame_count;
    uint32_t data_frame_count;
    uint32_t other_frame_count;

    /* 最近一帧 */
    uint32_t last_rx_can_id;
    uint8_t  last_rx_dlc;
    uint8_t  last_rx_data[8];
    uint8_t  rx_data_len;

    /* 最近4帧历史 */
    uint8_t  history_write_idx;
    uint8_t  history_count;
    IR_Debug_RxHistory_t rx_history[IR_DEBUG_RX_HISTORY_SIZE];

    /* 任务状态 */
    uint8_t  task_state;
    uint8_t  online_count;
    uint8_t  total_modules;

    /* 各模块debug (id + last_rx_time) */
    struct {
        uint8_t  module_id;
        uint32_t last_rx_time;
    } mods[IR_HOST_MAX_MODULES];
    uint8_t  mod_count;

    /* 投票debug */
    uint8_t  voted_data[8];
    uint8_t  voted_dlc;
    uint8_t  voted_agree_count;
    uint8_t  voted_total_count;
    uint32_t vote_count;
    uint8_t  voted_valid;

    /* 发送debug */
    uint32_t tx_success_count;
    uint32_t tx_fail_count;
} IR_Debug_Data_t;

extern IR_Host_Context_t ir_host_context;
extern IR_Debug_Data_t   ir_debug;

/* ======================== 核心 API ======================== */

bool  IR_Init(CAN_HandleTypeDef *hcan, uint8_t *module_ids, uint8_t count);
bool  IR_Send(uint8_t module_id, uint8_t *data, uint8_t length);
bool  IR_SendRetry(uint8_t module_id, uint8_t *data, uint8_t length, uint8_t retry);
bool  IR_SendAsync(uint8_t module_id, uint8_t *data, uint8_t length);
bool  IR_Read(uint8_t module_id, uint8_t *data, uint8_t *length);
bool  IR_ReadVoted(uint8_t *data, uint8_t *length);
bool  IR_IsOnline(uint8_t module_id);
void  IR_OnCanRx(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);

/* ======================== 辅助 API ======================== */

bool  IR_AddModule(uint8_t module_id);
bool  IR_RemoveModule(uint8_t module_id);
uint8_t IR_GetOnlineCount(void);
uint8_t IR_GetModuleCount(void);
IR_Host_TaskState_t IR_GetTaskState(void);
void  IR_ForceRediscover(void);
IR_Data_CheckResult_t IR_CheckData(uint8_t module_id);
IR_Vote_Result_t* IR_GetVoteResult(void);
bool  IR_PingTimeout(uint8_t module_id, uint32_t timeout_ms);
bool  IR_ResetTimeout(uint8_t module_id, uint32_t timeout_ms);

#endif
