#include "infrared.h"
#include "stm32f4xx_hal.h"
#include <string.h>


// 定时器句柄，假设已经通过CubeMX配置好TIM1
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;

// 接收8+1字节数据
uint8_t received_data[9];
uint8_t bit_index = 0;
uint32_t last_capture_time = 0;
uint8_t receiving = 0;

// 发送状态机上下文
static IR_TX_Context_t tx_context = {0};

// 发送引脚定义
#define IR_TX_GPIO_PORT     GPIOB
#define IR_TX_GPIO_PIN      GPIO_PIN_0
#define IR_TX_PWM_CHANNEL   TIM_CHANNEL_1

// 内部辅助函数声明
static inline void IR_TX_SetHigh(void);
static inline void IR_TX_SetLow(void);
static void IR_TX_SetNextTimer(uint16_t delay_us);

// 初始化红外发射端
void IR_Init(void)
{
    // 启动输入捕获，用于接收信号
    HAL_TIM_IC_Start_IT(&htim2, IC_CHANNEL);

    // 确保发送引脚初始为低电平
    IR_TX_SetLow();

    // 初始化发送状态机
    tx_context.state = IR_TX_IDLE;
    tx_context.busy = false;
}

// 设置红外输出高电平(载波)
static inline void IR_TX_SetHigh(void)
{
    // 启动PWM输出载波
    HAL_TIM_PWM_Start(&htim1, IR_TX_PWM_CHANNEL);
    HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_SET);
}

// 设置红外输出低电平(空闲)
static inline void IR_TX_SetLow(void)
{
    // 停止PWM输出
    HAL_TIM_PWM_Stop(&htim1, IR_TX_PWM_CHANNEL);
    HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_RESET);
}

// 设置下一次定时器中断时间
static void IR_TX_SetNextTimer(uint16_t delay_us)
{
    // 设置定时器计数器为0，并设置自动重装载值
    __HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim1, delay_us);

    // 清除更新中断标志
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_UPDATE);

    // 使能更新中断
    __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);

    // 启动定时器
    HAL_TIM_Base_Start_IT(&htim1);
}

// 非阻塞发送数据
// 返回: true-启动成功, false-发送忙
bool IR_SendData(uint8_t *data, uint8_t length)
{
    if (length > 8) return false;
    if (tx_context.busy) return false;

    // 计算CRC
    uint8_t crc = IR_CRC8(data, length);

    // 准备数据帧
    memcpy(tx_context.frame, data, length);
    tx_context.frame[8] = crc;
    tx_context.total_bytes = 9;
    tx_context.byte_index = 0;
    tx_context.bit_index = 7;  // 从高位开始发送
    tx_context.busy = true;

    // 启动状态机 - 发送起始脉冲
    tx_context.state = IR_TX_START_PULSE;
    IR_TX_SetHigh();
    IR_TX_SetNextTimer(START_PULSE_LEN);

    return true;
}

// 检查发送是否忙
bool IR_IsTXBusy(void)
{
    return tx_context.busy;
}

// 定时器中断回调 - 状态机核心
// 在 HAL_TIM_PeriodElapsedCallback 中调用，传入触发中断的定时器句柄
void IR_TX_TimerCallback(TIM_HandleTypeDef *htim)
{
    // 检查是否是发送定时器 TIM1
    if (htim->Instance != TIM1) return;

    // 检查是否是更新中断
    if (!(__HAL_TIM_GET_FLAG(htim, TIM_FLAG_UPDATE) &&
          __HAL_TIM_GET_IT_SOURCE(htim, TIM_IT_UPDATE))) {
        return;
    }

    // 清除中断标志
    __HAL_TIM_CLEAR_IT(htim, TIM_IT_UPDATE);

    // 状态机处理
    switch (tx_context.state) {
        case IR_TX_IDLE:
            // 空闲状态，停止定时器
            HAL_TIM_Base_Stop_IT(htim);
            break;

        case IR_TX_START_PULSE:
            // 起始脉冲结束，进入起始间隔
            tx_context.state = IR_TX_START_SPACE;
            IR_TX_SetLow();
            IR_TX_SetNextTimer(START_SPACE_LEN);
            break;

        case IR_TX_START_SPACE:
            // 起始间隔结束，开始发送数据
            tx_context.state = IR_TX_DATA_HIGH;
            IR_TX_SetHigh();
            IR_TX_SetNextTimer(BIT_ONE_HIGH);  // 数据位高电平都是560us
            break;

        case IR_TX_DATA_HIGH:
            // 数据位高电平结束，进入低电平
            {
                uint8_t current_byte = tx_context.frame[tx_context.byte_index];
                uint8_t current_bit = (current_byte >> tx_context.bit_index) & 0x01;

                tx_context.state = IR_TX_DATA_LOW;
                IR_TX_SetLow();

                // 根据当前位设置低电平时间
                if (current_bit) {
                    IR_TX_SetNextTimer(BIT_ONE_LOW);
                } else {
                    IR_TX_SetNextTimer(BIT_ZERO_LOW);
                }
            }
            break;

        case IR_TX_DATA_LOW:
            // 数据位低电平结束，准备下一个位
            {
                // 位索引递减(高位先发送)
                if (tx_context.bit_index > 0) {
                    tx_context.bit_index--;
                } else {
                    // 当前字节发送完成，进入下一个字节
                    tx_context.bit_index = 7;
                    tx_context.byte_index++;
                }

                // 检查是否所有字节都发送完成
                if (tx_context.byte_index >= tx_context.total_bytes) {
                    // 发送完成
                    tx_context.state = IR_TX_STOP;
                    IR_TX_SetLow();
                    tx_context.busy = false;
                    HAL_TIM_Base_Stop_IT(htim);
                } else {
                    // 继续发送下一个位
                    tx_context.state = IR_TX_DATA_HIGH;
                    IR_TX_SetHigh();
                    IR_TX_SetNextTimer(BIT_ONE_HIGH);
                }
            }
            break;

        case IR_TX_STOP:
        default:
            // 停止状态
            IR_TX_SetLow();
            tx_context.busy = false;
            tx_context.state = IR_TX_IDLE;
            HAL_TIM_Base_Stop_IT(htim);
            break;
    }
}

// CRC8计算
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

// 输入捕获回调（用于接收数据）
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
        uint32_t capture_value = HAL_TIM_ReadCapturedValue(htim, IC_CHANNEL);

        if (last_capture_time != 0) {
            uint32_t pulse_duration;
            if (capture_value < last_capture_time) {
                pulse_duration = (0xFFFFFFFF - last_capture_time) + capture_value;
            } else {
                pulse_duration = capture_value - last_capture_time;
            }

            // 检测起始信号
            if (pulse_duration > 10000) {
                receiving = 1;
                bit_index = 0;
                for (int i = 0; i < 9; i++) {
                    received_data[i] = 0;
                }
                last_capture_time = capture_value;
                return;
            }

            if (receiving == 0) {
                last_capture_time = capture_value;
                return;
            }

            // 解析 bit
            uint8_t bit_value;
            if (pulse_duration >= 1500 && pulse_duration < 2500) {
                bit_value = 0;
            } else if (pulse_duration >= 500 && pulse_duration < 1200) {
                bit_value = 1;
            } else {
                last_capture_time = capture_value;
                return;
            }

            if (bit_index < 72) {
                uint8_t byte_idx = bit_index / 8;
                uint8_t bit_pos = 7 - (bit_index % 8);
                received_data[byte_idx] |= (bit_value << bit_pos);
                bit_index++;

                if (bit_index >= 72) {
                    receiving = 0;
                    IR_ReceiveData();
                }
            }
        }

        last_capture_time = capture_value;
    }
}

// 数据接收处理
void IR_ReceiveData(void)
{
    uint8_t crc_received = received_data[8];
    uint8_t crc_calculated = IR_CRC8(received_data, 8);

    if (crc_received == crc_calculated) {
        // 数据校验成功
        // received_data[0] 到 received_data[7] 是有效数据
    } else {
        // CRC 错误
    }
}

// 重置接收数据缓冲区
void IR_ResetBuffer(void)
{
    for (int i = 0; i < 9; i++) {
        received_data[i] = 0;
    }
    bit_index = 0;
    receiving = 0;
}
