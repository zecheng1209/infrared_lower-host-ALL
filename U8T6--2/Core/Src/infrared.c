

//红外模块实现  具有解析can数据帧、ack确认、crc校验、超时重传等功能
#include "infrared.h"
#include "stm32f1xx_hal.h"
#include <string.h>

extern TIM_HandleTypeDef htim1;  // 用于 38KHz PWM 载波 (PA8 - TIM1_CH1)
extern TIM_HandleTypeDef htim2;  // 用于接收输入捕获 (PA15 - TIM2_CH1)，该芯片不支持上下沿同时捕获，程序中设置的为上升沿捕获。
extern TIM_HandleTypeDef htim3;  // 用于发送时序控制
extern CAN_HandleTypeDef hcan;

uint8_t received_data[9];
uint8_t bit_index = 0;
uint32_t last_capture_time = 0;
uint8_t receiving = 0;
uint32_t rx_last_activity_time = 0;

static IR_TX_Context_t tx_context = {IR_TX_IDLE};

// 自发自收过滤机制：发送冷却期
volatile uint32_t ir_last_tx_complete_time = 0;  // 最后发送完成时间（数据帧）
volatile uint32_t ir_last_ack_tx_time = 0;       // 最后ACK/NACK发送完成时间
volatile uint8_t ir_rx_ignore_self = 0;           // 忽略自收标志
volatile uint8_t ir_sending_ack = 0;              // 正在发送ACK/NACK的标志
volatile uint32_t ir_rx_guard_until = 0;          // 接收保护期截止时间（ms），TX完成后5ms内忽略捕获

// ===== 调试专用：接收数据快照系统（定义IR_DEBUG启用） =====
#ifdef IR_DEBUG
uint8_t ir_debug_snapshot[IR_DEBUG_SNAPSHOT_COUNT][IR_DEBUG_DATA_LEN] = {0};
volatile uint8_t ir_debug_snapshot_index = 0;
volatile uint8_t ir_debug_snapshot_valid[IR_DEBUG_SNAPSHOT_COUNT] = {0};
volatile uint32_t ir_debug_snapshot_time[IR_DEBUG_SNAPSHOT_COUNT] = {0};
volatile uint8_t ir_debug_frame_type[IR_DEBUG_SNAPSHOT_COUNT] = {0};
volatile uint16_t ir_debug_rx_total_count = 0;
#endif

static void IR_TX_SetNextTimer(uint16_t delay_us)
{
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim3, delay_us);
    __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_UPDATE);
}
 
void IR_Init(void)
{
    HAL_TIM_IC_Start_IT(&htim2, IC_CHANNEL);
    HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_RESET);
    tx_context.state = IR_TX_IDLE;
    tx_context.busy = false;
    rx_last_activity_time = HAL_GetTick();
}

bool IR_SendData(uint8_t *data, uint8_t length)
{
    if (length > 8) return false;
    if (tx_context.busy) return false;

    uint32_t time_since_last_tx = HAL_GetTick() - tx_context.last_tx_time;
    if (time_since_last_tx < IR_TX_FRAME_INTERVAL_MS) {
        return false;
    }

    uint8_t crc = IR_CRC8(data, length);
    memcpy(tx_context.frame, data, length);
    tx_context.frame[length] = crc;
    tx_context.total_bytes = length + 1;
    tx_context.byte_index = 0;
    tx_context.bit_index = 7;
    tx_context.busy = true;

    // 设置自发自收过滤：标记即将发送
    ir_rx_ignore_self = 1;

    tx_context.state = IR_TX_START_PULSE;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    IR_TX_SetNextTimer(START_PULSE_LEN);
    HAL_TIM_Base_Start_IT(&htim3);
    HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_SET);
    return true;
}

bool IR_IsTXBusy(void)
{
    return tx_context.busy;
}

void IR_TX_TimerCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM3) return;
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);
    
    switch (tx_context.state) {
        case IR_TX_START_PULSE:
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_RESET);
            tx_context.state = IR_TX_START_SPACE;
            IR_TX_SetNextTimer(START_SPACE_LEN);
            break;

        case IR_TX_START_SPACE:
            HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
            tx_context.state = IR_TX_DATA_HIGH;
            IR_TX_SetNextTimer(BIT_ONE_HIGH);
            break;

        case IR_TX_DATA_HIGH:
            {
                uint8_t current_bit = (tx_context.frame[tx_context.byte_index] >> tx_context.bit_index) & 0x01;
                HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
                HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_RESET);
                tx_context.state = IR_TX_DATA_LOW;
                IR_TX_SetNextTimer(current_bit ? BIT_ONE_LOW : BIT_ZERO_LOW);
            }
            break;

        case IR_TX_DATA_LOW:
            if (tx_context.bit_index > 0) {
                tx_context.bit_index--;
            } else {
                tx_context.bit_index = 7;
                tx_context.byte_index++;
            }

            if (tx_context.byte_index >= tx_context.total_bytes) {
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                tx_context.state = IR_TX_STOP_PULSE;
                IR_TX_SetNextTimer(BIT_ONE_HIGH);
            } else {
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
                tx_context.state = IR_TX_DATA_HIGH;
                IR_TX_SetNextTimer(BIT_ONE_HIGH);
            }
            break;

        case IR_TX_STOP_PULSE: {
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_RESET);
            tx_context.state = IR_TX_IDLE;
            tx_context.busy = false;
            tx_context.last_tx_time = HAL_GetTick();

            // 记录发送完成时间，用于自发自收过滤
            uint32_t now = HAL_GetTick();
            ir_last_tx_complete_time = now;

            // 如果是ACK/NACK帧，单独记录时间（ACK冷却期更短）
            if (ir_sending_ack) {
                ir_last_ack_tx_time = now;
                ir_sending_ack = 0;  // 清除标志
            }

            // ★ 发送完成后立即重置接收器状态 + 5ms保护期
            // 发送期间接收器被TX信号污染，不重置会漏掉对方回的ACK
            // 保护期防止TX尾抖被误判为起始信号
            receiving = 0;
            bit_index = 0;
            last_capture_time = 0;
            memset(received_data, 0, 9);
            ir_rx_guard_until = now + IR_TX_GUARD_MS;

            HAL_TIM_Base_Stop_IT(&htim3);
            break;
        }

        default:
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_RESET);
            tx_context.busy = false;
            tx_context.state = IR_TX_IDLE;
            HAL_TIM_Base_Stop_IT(&htim3);
            break;
    }
}

uint8_t IR_CRC8(uint8_t *data, uint8_t length)
{
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_1) return;

    uint32_t capture_value = HAL_TIM_ReadCapturedValue(htim, IC_CHANNEL);
    rx_last_activity_time = HAL_GetTick();

    if (last_capture_time != 0) {
        uint32_t pulse_duration = (capture_value < last_capture_time) ?
                         (0xFFFF - last_capture_time + capture_value) :
                         (capture_value - last_capture_time);

        if (!receiving) {
            // ★ TX保护期检查：发送完成后5ms内忽略信号，防止尾抖触发
            if (HAL_GetTick() < ir_rx_guard_until) {
                last_capture_time = capture_value;
                return;
            }
            // 检测起始信号：4.5ms低电平(载波) + 2.25ms高电平(空闲)
            // 总共约6.75ms (6750us)
            uint32_t start_total = START_PULSE_LEN + START_SPACE_LEN; // 6750us
            uint32_t tolerance = IR_PULSE_TOLERANCE_US * 3; // 起始信号容差稍大
            if (pulse_duration >= (start_total - tolerance) &&
                pulse_duration <= (start_total + tolerance)) {
                receiving = 1;
                bit_index = 0;
                memset(received_data, 0, 9);
            }
        } else {
            // 正在接收中
            if (bit_index >= 72) {
                last_capture_time = capture_value;
                return;
            }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // 数据位判断：位0=280us高+280us低 总计560us，位1=280us高+840us低 总计1120us
            // 红外接收模块是接收到38kHZ的红外信息，就会将其中一个口设置为高电平，低电平则设置为低电平。
            // 本模块通过捕获这个口的高低变化，判断是位0还是位1。
						// 我所使用的接收模块最精确基本就在二三百us左右，再低会难以识别。
            // 这里检测的是整个位周期（高电平+低电平）。这里是检测两个上升沿（也就是红外灯由低变高）的时间，
            // 若使用程序在 几百微秒内翻转检测规则，即能够检测上升沿以及下降沿，容易出现问题，故未采用
            // 位0: 约560us (280+280) 最好情况，全是0位，起始信号6.75ms 72 × 560 µs 40.5 ms  合计约为47ms
            // 位1: 约1120us (280+840)最坏情况，全是1位，起始信号6.75ms 72 × 1120 µs 80ms以及结束位 ~87.67 ms
						// 如果环境存在各种干扰（强光源、激光模块、信号重叠与反射），可以试试恢复到正常的NEC的时序，容错更高
            uint32_t bit0_total = BIT_ZERO_HIGH + BIT_ZERO_LOW;  // 560us
            uint32_t bit1_total = BIT_ONE_HIGH + BIT_ONE_LOW;     // 1120us
            uint32_t tolerance = IR_PULSE_TOLERANCE_US;

            if (pulse_duration >= (bit0_total - tolerance) &&
                pulse_duration <= (bit0_total + tolerance)) {
                // 位0不需要设置，缓冲区已初始化为0
                bit_index++;
            }
            else if (pulse_duration >= (bit1_total - tolerance) &&
                     pulse_duration <= (bit1_total + tolerance)) {
                received_data[bit_index / 8] |= (1 << (7 - (bit_index % 8)));
                bit_index++;
            }
            else if (pulse_duration > IR_MAX_PULSE_US) {
                receiving = 0;
                bit_index = 0;
                last_capture_time = capture_value;
                return;
            }

            // 接收完成
            if (bit_index >= 72) {
                receiving = 0;
                IR_ReceiveData();
                last_capture_time = 0;
                return;
            }
        }
    }
    last_capture_time = capture_value;
}

// 红外接收完成标志（供主循环查询）
volatile uint8_t ir_rx_complete_flag = 0;
volatile uint8_t ir_ack_received_flag = 0; // ACK接收标志
volatile uint8_t ir_ack_status = 0;        // ACK状态: 0=无, 1=ACK, 2=NACK

void IR_ReceiveData(void) {
    // ===== 智能自发自收过滤（基于帧类型） =====

    // 先预判帧类型（不修改数据）
    uint8_t is_ack_frame = (received_data[0] == IR_ACK_MAGIC && received_data[1] == IR_ACK_MAGIC);
    uint8_t is_nack_frame = (received_data[0] == IR_NACK_MAGIC && received_data[1] == IR_NACK_MAGIC);
    uint8_t is_control_frame = is_ack_frame || is_nack_frame;

    if (ir_rx_ignore_self) {
        uint32_t now = HAL_GetTick();

        if (is_control_frame) {
            // ACK/NACK帧：使用较短的冷却期（40ms）
            // 因为：①只有2字节，发送时间短 ②需要尽快接收对方的确认
            uint32_t time_since_ack_tx = now - ir_last_ack_tx_time;
            if (time_since_ack_tx < IR_ACK_FILTER_MS) {
                // 在ACK冷却期内，忽略这个ACK/NACK（可能是自己发的回声）
                return;
            }
        } else {
            // 数据帧：使用标准冷却期（80ms）
            uint32_t time_since_tx = now - ir_last_tx_complete_time;
            if (time_since_tx < IR_RX_SELF_FILTER_MS) {
                // 在冷却期内，忽略这个数据（很可能是自己发的）
                return;
            }
        }

        // 所有冷却期都结束了，清除忽略标志
        ir_rx_ignore_self = 0;
    }

    // ===== 调试专用：立即创建数据快照（定义IR_DEBUG启用） =====
#ifdef IR_DEBUG
    {
        uint8_t idx = ir_debug_snapshot_index;
        memcpy(ir_debug_snapshot[idx], received_data, IR_DEBUG_DATA_LEN);
        ir_debug_snapshot_time[idx] = HAL_GetTick();
        ir_debug_snapshot_valid[idx] = 1;
        if (is_ack_frame) {
            ir_debug_frame_type[idx] = 1;
        } else if (is_nack_frame) {
            ir_debug_frame_type[idx] = 2;
        } else {
            ir_debug_frame_type[idx] = 0;
        }
        ir_debug_rx_total_count++;
        ir_debug_snapshot_index = (idx + 1) % IR_DEBUG_SNAPSHOT_COUNT;
    }
#endif

    // ===== 正常处理接收到的帧 =====
    // 检查是否是ACK帧
    if (is_ack_frame) {
        ir_ack_status = 1;  // 收到ACK
        ir_ack_received_flag = 1;
        return;
    }
    // 检查是否是NACK帧
    if (is_nack_frame) {
        ir_ack_status = 2;  // 收到NACK
        ir_ack_received_flag = 1;
        return;
    }
    // 普通数据帧，设置接收完成标志
    ir_rx_complete_flag = 1;
}

void IR_ResetBuffer(void)
{
    memset(received_data, 0, 9);
    bit_index = 0;
    receiving = 0;
}

// 发送ACK帧（非阻塞方式，推荐用于主循环）
bool IR_SendDataAck_NonBlocking(uint8_t status)
{
    uint8_t ack_frame[2];
    if (status == 1) {
        ack_frame[0] = IR_ACK_MAGIC;
        ack_frame[1] = IR_ACK_MAGIC;
    } else {
        ack_frame[0] = IR_NACK_MAGIC;
        ack_frame[1] = IR_NACK_MAGIC;
    }

    // 标记正在发送ACK/NACK
    ir_sending_ack = 1;

    return IR_SendData(ack_frame, 2);
}

void IR_CheckRxTimeout(void)
{
    if (receiving && (HAL_GetTick() - rx_last_activity_time > IR_RX_TIMEOUT_MS)) {
        receiving = 0;
        bit_index = 0;
        last_capture_time = 0;
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM3) {
        IR_TX_TimerCallback(htim);
    }
}

// ===== 调试辅助函数（定义IR_DEBUG启用） =====
#ifdef IR_DEBUG
uint8_t IR_DebugGetLatestSnapshot(void)
{
    int8_t idx = ((int8_t)ir_debug_snapshot_index - 1 + IR_DEBUG_SNAPSHOT_COUNT) % IR_DEBUG_SNAPSHOT_COUNT;
    for (uint8_t i = 0; i < IR_DEBUG_SNAPSHOT_COUNT; i++) {
        if (ir_debug_snapshot_valid[idx]) {
            return (uint8_t)idx;
        }
        idx = ((int8_t)idx - 1 + IR_DEBUG_SNAPSHOT_COUNT) % IR_DEBUG_SNAPSHOT_COUNT;
    }
    return 0xFF;
}

bool IR_DebugCopySnapshot(uint8_t snapshot_index, uint8_t *buffer, uint8_t length)
{
    if (snapshot_index >= IR_DEBUG_SNAPSHOT_COUNT) return false;
    if (!ir_debug_snapshot_valid[snapshot_index]) return false;
    if (length < IR_DEBUG_DATA_LEN) return false;
    memcpy(buffer, ir_debug_snapshot[snapshot_index], IR_DEBUG_DATA_LEN);
    return true;
}

void IR_DebugClearSnapshots(void)
{
    for (uint8_t i = 0; i < IR_DEBUG_SNAPSHOT_COUNT; i++) {
        ir_debug_snapshot_valid[i] = 0;
        memset(ir_debug_snapshot[i], 0, IR_DEBUG_DATA_LEN);
    }
    ir_debug_snapshot_index = 0;
    ir_debug_rx_total_count = 0;
}

const char* IR_DebugGetFrameTypeStr(uint8_t snapshot_index)
{
    if (snapshot_index >= IR_DEBUG_SNAPSHOT_COUNT) return "INVALID";
    switch (ir_debug_frame_type[snapshot_index]) {
        case 0: return "DATA";
        case 1: return "ACK";
        case 2: return "NACK";
        default: return "UNKNOWN";
    }
}
#endif

volatile uint32_t can_tx_success = 0;
volatile uint32_t can_tx_fail = 0;
void CAN_SendFrame(uint32_t can_id, uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t tx_mailbox;
    
    tx_header.StdId = can_id;
    tx_header.ExtId = 0;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;
    tx_header.TransmitGlobalTime = DISABLE;
    
    // 等待邮箱可用（最多重试3次，每次1ms）
    uint8_t retry = 3;
    while (retry--) {
        if (HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &tx_mailbox) == HAL_OK) {
            can_tx_success++;
            return;
        }
        HAL_Delay(1);
    }
    can_tx_fail++;
}
