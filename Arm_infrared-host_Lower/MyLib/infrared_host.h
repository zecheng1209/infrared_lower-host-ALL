#ifndef __INFRARED_HOST_H
#define __INFRARED_HOST_H

#include "stm32f4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include <stdbool.h>
#include <string.h>

/* ================================================================
 *                        可调参数宏
 * ================================================================ */

#define IR_HOST_CAN_TIMEOUT_MS     500      ///< CAN发送超时(ms)
#define IR_HOST_MAX_RETRY_COUNT    3        ///< 最大重传次数
#define IR_HOST_FRAME_INTERVAL_MS  200      ///< CAN帧最小发送间隔(ms)

#define IR_ACK_MAGIC               0xA5     ///< ACK帧魔术字节
#define IR_NACK_MAGIC              0x5A     ///< NACK帧魔术字节

#define IR_HOST_DEFAULT_MODULE_ID  0x10     ///< 默认模块ID       //与电机做区分从10开始

#define IR_HOST_RX_QUEUE_SIZE      20       ///< 接收队列深度
#define IR_HOST_MAX_MODULES        8        ///< 最大管理模块数
#define IR_HOST_POLL_INTERVAL_MS   300      ///< 轮询间隔(ms)
#define IR_HOST_ONLINE_TIMEOUT_MS  3000     ///< 在线超时(ms)
#define IR_HOST_DATA_STALE_MS      500      ///< 数据过期阈值(ms)
#define IR_HOST_MAX_CRC_ERRORS     5        ///< 最大CRC错误计数
#define IR_HOST_VOTE_WINDOW_MS     50       ///< 投票收集窗口(ms)

/* ================================================================
 *                     CAN ID 编码宏，结合了帧类型和模块id的编码，
                               所以在后文都红外的时候一定注意can帧的类型以及模块id的整合。！！！！！
 * 方案: [10:8]=帧类型(1=CMD/2=DATA/3=ACK/4=RELIABLE_DATA), [7:0]=模块ID
 * 示例:
         1》module_id=0x10 → CMD=0x110, DATA=0x210, ACK=0x310, RDATA=0x410
         2》          0x20       0x120，     0x220，    0x320，     0x420
 * ================================================================ */

#define IR_HOST_CAN_TYPE_CMD           1
#define IR_HOST_CAN_TYPE_DATA          2
#define IR_HOST_CAN_TYPE_ACK           3
#define IR_HOST_CAN_TYPE_RELIABLE_DATA 4   ///< 可靠数据帧：下位机等红外投递成功后才回CAN ACK

#define IR_HOST_CAN_ID_BUILD(type, mid)     (((uint32_t)(type) << 8) | (uint32_t)(mid))
#define IR_HOST_CAN_ID_GET_TYPE(can_id)     ((uint8_t)((can_id) >> 8))
#define IR_HOST_CAN_ID_GET_MODULE(can_id)   ((uint8_t)((can_id) & 0xFF))

#define IR_HOST_IS_CMD_FRAME(can_id)           (IR_HOST_CAN_ID_GET_TYPE(can_id) == IR_HOST_CAN_TYPE_CMD)
#define IR_HOST_IS_DATA_FRAME(can_id)          (IR_HOST_CAN_ID_GET_TYPE(can_id) == IR_HOST_CAN_TYPE_DATA)
#define IR_HOST_IS_ACK_FRAME(can_id)           (IR_HOST_CAN_ID_GET_TYPE(can_id) == IR_HOST_CAN_TYPE_ACK)
#define IR_HOST_IS_RELIABLE_DATA_FRAME(can_id) (IR_HOST_CAN_ID_GET_TYPE(can_id) == IR_HOST_CAN_TYPE_RELIABLE_DATA)

#define IR_HOST_CAN_ID_COMMAND(mid)        IR_HOST_CAN_ID_BUILD(IR_HOST_CAN_TYPE_CMD, mid)
#define IR_HOST_CAN_ID_DATA(mid)           IR_HOST_CAN_ID_BUILD(IR_HOST_CAN_TYPE_DATA, mid)
#define IR_HOST_CAN_ID_ACK(mid)            IR_HOST_CAN_ID_BUILD(IR_HOST_CAN_TYPE_ACK, mid)
#define IR_HOST_CAN_ID_RELIABLE_DATA(mid)  IR_HOST_CAN_ID_BUILD(IR_HOST_CAN_TYPE_RELIABLE_DATA, mid)

/* ================================================================
 *                        枚举
 * ================================================================ */

typedef enum {
    IR_HOST_CMD_PING = 0x01,        ///< 心跳探测
    IR_HOST_CMD_SEND_DATA = 0x02,   ///< 发送数据
    IR_HOST_CMD_READ_STATUS = 0x03, ///< 读取状态
    IR_HOST_CMD_RESET = 0x04,       ///< 复位从机
    IR_HOST_CMD_READ_SENSOR = 0x05, ///< 读取传感器
    IR_HOST_CMD_READ_ALL = 0x06     ///< 读取全部
} IR_Host_Command_t;

typedef enum {
    IR_HOST_STATUS_IDLE = 0x00,     ///< 空闲
    IR_HOST_STATUS_SENDING = 0x01,  ///< 发送中
    IR_HOST_STATUS_WAIT_ACK = 0x02, ///< 等待ACK
    IR_HOST_STATUS_SUCCESS = 0x03,  ///< 成功
    IR_HOST_STATUS_TIMEOUT = 0x04,  ///< 超时
    IR_HOST_STATUS_NACK = 0x05,     ///< 收到NACK
    IR_HOST_STATUS_ERROR = 0x06     ///< 错误
} IR_Host_Status_t;

typedef enum {
    IR_HOST_TASK_STATE_INIT = 0,    ///< 初始化
    IR_HOST_TASK_STATE_DISCOVERY,   ///< 发现阶段
    IR_HOST_TASK_STATE_RUNNING,     ///< 正常运行
    IR_HOST_TASK_STATE_ERROR        ///< 错误
} IR_Host_TaskState_t;

typedef enum {
    IR_DATA_CHECK_OK = 0,           ///< 数据新鲜且有效
    IR_DATA_CHECK_STALE,            ///< 数据过期
    IR_DATA_CHECK_CRC_ERR,          ///< 数据校验错误
    IR_DATA_CHECK_OFFLINE,          ///< 模块离线
    IR_DATA_CHECK_NO_MODULE         ///< 模块不存在
} IR_Data_CheckResult_t;

/* ================================================================
 *                  投票结果
 * ================================================================ */

typedef struct {
    uint8_t data[8];            ///< 投票得出的数据
    uint8_t dlc;                ///< 数据长度
    bool valid;                 ///< 投票结果是否有效
    uint8_t agree_count;        ///< 同意该数据的模块数
    uint8_t total_count;        ///< 参与投票的模块数
    uint32_t timestamp;         ///< 投票完成时间
} IR_Vote_Result_t;

/**
 * @brief  IR_ReadAll 返回的单条去重结果
 */
typedef struct {
    uint8_t data[8];                ///< 去重后的数据
    uint8_t dlc;                    ///< 数据长度
    uint8_t matched_module_count;   ///< 有多少模块收到了相同数据
} IR_ReadAllResult_t;

/* ================================================================
 *                  内部类型 
 * ================================================================ */

typedef struct {
    uint32_t can_id;
    uint8_t module_id;
    uint8_t data[8];
    uint8_t dlc;
    uint32_t timestamp;
} IR_Host_RxFrame_t;

typedef struct {
    uint8_t module_id;
    IR_Host_Status_t status;
    bool valid;
    uint8_t length;
    uint8_t data[8];
    uint32_t timestamp;
} IR_Host_ResponseFrame_t;

typedef struct {
    uint8_t raw_data[8];
    uint32_t update_timestamp;
    bool valid;
    uint8_t crc_error_count;
    uint32_t total_rx_count;
    uint32_t total_crc_error_count;
} IR_Module_DataCache_t;

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
    CAN_HandleTypeDef *hcan;
    uint32_t can_timeout_ms;
    uint8_t  max_retry;
    uint32_t poll_interval_ms;
    uint32_t online_timeout_ms;
    // 投票
    bool vote_window_active;
    uint32_t vote_window_start_time;
    IR_Vote_Result_t vote_result;
    uint32_t total_votes;
} IR_Host_Context_t;

extern IR_Host_Context_t ir_host_context;

/* ================================================================
                           核心 API 
 * ================================================================ */

/**
 * @brief  初始化红外主机并启动后台任务
 *
 * @param[in]  hcan       CAN句柄 (如 &hcan1)，CAN硬件需已由工程初始化
 * @param[in]  module_ids 模块ID数组, 如 (uint8_t[]){0x10, 0x11}
 * @param[in]  count      模块数量
 * @return true  成功
 * @return false 失败(参数无效)
 *
 * @note  不操作CAN硬件，工程需自行初始化CAN:
 *        1) CanFilter_Init(&hcan1) 或 HAL_CAN_ConfigFilter
 *        2) HAL_CAN_Start(&hcan1)
 *        3) HAL_CAN_ActivateNotification(&hcan1, ...)
 *
 * @code
 *   IR_Init(&hcan1, (uint8_t[]){0x10}, 1);
 * @endcode
 */
bool IR_Init(CAN_HandleTypeDef *hcan, uint8_t *module_ids, uint8_t count);


/**
 * @brief  阻塞发送 — 等ACK才返回，失败自动重试
 *
 * @param[in]  module_id 模块ID，模块id的使用在can包发送前的id帧里体现
 * @param[in]  data      发送数据
 * @param[in]  length    数据长度(1~8)
 * @return true  成功
 * @return false 失败(超时/重试耗尽)
 *
 * @warning 阻塞! 不可在中断中调用
 *
 * @code
 *   uint8_t payload[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
 *   if (IR_Send(0x10, payload, 8)) { // 发给0x10，等ACK确认 }
 * @endcode
 */
bool IR_Send(uint8_t module_id, uint8_t *data, uint8_t length);


/**
 * @brief  非阻塞发送 — 立即返回
 *
 * @param[in]  module_id 模块ID
 * @param[in]  data      发送数据
 * @param[in]  length    数据长度(1~8)
 * @return true  已提交
 * @return false 失败(模块忙/CAN失败)
 *
 * @code
 *   uint8_t payload[4] = {0xAA, 0xBB, 0xCC, 0xDD};
 *   IR_SendAsync(0x10, payload, 4); // 立即返回，不等ACK
 * @endcode
 */
bool IR_SendAsync(uint8_t module_id, uint8_t *data, uint8_t length);


/**
 * @brief  读取指定模块的原始数据（无投票）
 *
 * 直接返回指定模块的最新缓存数据，不经过投票筛选。
 * 适合需要判断信号来源方向的场景（如按模块ID区分前后左右）。
 *
 * @param[in]   module_id 模块ID
 * @param[out]  data      输出缓冲区(8字节), 可NULL
 * @param[out]  length    输出数据实际长度, 可NULL
 * @return true  数据有效
 * @return false 模块不存在/数据无效
 *
 * @code
 *   uint8_t buf[8], len;
 *   if (IR_Read(0x10, buf, &len)) { // 读取前方模块数据 }
 * @endcode
 */
bool IR_Read(uint8_t module_id, uint8_t *data, uint8_t *length);


/**
 * @brief  读取投票筛选后的数据（多模块多数派）
 *
 * 多个模块同时接收同一IR信号时，取多数派一致的数据作为可信结果。
 * 仅1个模块在线时直接返回该模块数据。
 * 与 IR_Read() 的区别：IR_Read 返回指定模块的原始数据（可判断方向），
 * IR_ReadVoted 返回多模块投票后的数据（可靠性更高但丢失方向信息）。
 *
 * @param[out]  data    输出缓冲区(8字节), 可NULL
 * @param[out]  length  输出数据长度, 可NULL
 * @return true  投票结果有效
 * @return false 无有效投票结果
 *
 * @code
 *   uint8_t buf[8], len;
 *   if (IR_ReadVoted(buf, &len)) {
 *       IR_Vote_Result_t *v = IR_GetVoteResult();
 *       // v->agree_count: 同意该数据的模块数
 *       // v->total_count: 参与投票的模块数
 *   }
 * @endcode
 */
bool IR_ReadVoted(uint8_t *data, uint8_t *length);


/**
 * @brief  收集所有在线模块的CRC校验通过数据，去重后返回
 *
 * 遍历所有模块，将 data_cache.valid 的缓存数据按内容去重。
 * 多个模块收到相同数据时合并为一条，matched_module_count 记录同意的模块数。
 * 与 IR_ReadVoted 的区别：ReadVoted 只返回多数派那一条；
 * ReadAll 返回所有唯一数据，适合需要知道接收到了"哪些不同消息"的场景。
 *
 * @param[out]  results     输出结果数组
 * @param[in]   max_results 数组最大容量
 * @return 实际去重后的条目数(0 = 无有效数据)
 *
 * @code
 *   IR_ReadAllResult_t results[4];
 *   uint8_t count = IR_ReadAll(results, 4);
 *   for (uint8_t i = 0; i < count; i++) {
 *       // results[i].data / .dlc / .matched_module_count
 *       printf("msg%d: %d modules agree\n", i, results[i].matched_module_count);
 *   }
 * @endcode
 */
uint8_t IR_ReadAll(IR_ReadAllResult_t *results, uint8_t max_results);


/**
 * @brief  判断模块是否在线
 *
 * @param[in]  module_id 模块ID
 * @return true  在线
 * @return false 离线或不存在
 *
 * @code
 *   if (IR_IsOnline(0x10)) { IR_Send(0x10, data, 8); }
 * @endcode
 */
bool IR_IsOnline(uint8_t module_id);


/**
 * @brief  CAN接收中断回调 — 在HAL回调中调用
 *
 * @param[in]  rx_header CAN接收头
 * @param[in]  rx_data   接收数据(最多8字节)
 *
 * @code
 *   void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
 *       CAN_RxHeaderTypeDef hdr; uint8_t data[8];
 *       if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, data) == HAL_OK)
 *           IR_OnCanRx(&hdr, data);
 *   }
 * @endcode
 */
void IR_OnCanRx(CAN_RxHeaderTypeDef *rx_header, uint8_t *rx_data);


/* ================================================================
 *            辅助 API 
 * ================================================================ */

bool IR_Ping(uint8_t module_id);
bool IR_Reset(uint8_t module_id);
bool IR_AddModule(uint8_t module_id);
bool IR_RemoveModule(uint8_t module_id);
IR_Data_CheckResult_t IR_CheckData(uint8_t module_id);
uint8_t IR_GetOnlineCount(void);
uint8_t IR_GetModuleCount(void);
IR_Host_TaskState_t IR_GetTaskState(void);
IR_Data_CheckResult_t IR_CheckAllModules(void);
void IR_ForceRediscover(void);

/* ================================================================
 *         多模块投票（辅助）
 * ================================================================ */

/**
 * @brief  获取投票结果详情（含同意数、参与数等）
 *
 * @code
 *   IR_Vote_Result_t *v = IR_GetVoteResult();
 *   if (v->valid && v->agree_count >= 2) { // 多数派确认 }
 * @endcode
 */
IR_Vote_Result_t* IR_GetVoteResult(void);

/* ================================================================
 *         精细控制
 * ================================================================ */

bool IR_SendRetry(uint8_t module_id, uint8_t *data, uint8_t length, uint8_t retry);
bool IR_SendCommand(uint8_t module_id, IR_Host_Command_t cmd, uint8_t *data, uint8_t length);
bool IR_ReadStatus(uint8_t module_id, uint8_t *status, uint32_t timeout_ms);
bool IR_PingTimeout(uint8_t module_id, uint32_t timeout_ms);
bool IR_ResetTimeout(uint8_t module_id, uint32_t timeout_ms);

/**
 * @brief  广播发送 — 所有下位机同时接收并转发红外
 *
 * 使用广播ID 0x00发送DATA帧，所有在线下位机都会收到
 * 并各自转发红外。不等ACK，立即返回（因为多个模块会
 * 各自按错峰延迟回ACK，无法统一等待）。
 *
 * @param[in]  data   发送数据
 * @param[in]  length 数据长度(1~8)
 * @return true  CAN发送成功
 * @return false CAN发送失败
 *
 * @code
 *   uint8_t payload[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
 *   IR_SendBroadcast(payload, 8); // 所有下位机同时转发红外
 * @endcode
 */
bool IR_SendBroadcast(uint8_t *data, uint8_t length);

void IR_StartTest(CAN_HandleTypeDef *hcan);

#endif
