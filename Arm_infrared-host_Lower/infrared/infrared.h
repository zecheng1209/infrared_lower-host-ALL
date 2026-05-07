#ifndef __INFRARED_H
#define __INFRARED_H

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdbool.h>

// 红外时序定义 (NEC协议)
#define IR_FREQUENCY    38000
#define START_PULSE_LEN 9000  // 起始信号：9ms高电平
#define START_SPACE_LEN 4500  // 起始信号：4.5ms低电平
#define BIT_ONE_HIGH    560   // 1的高电平：560us
#define BIT_ONE_LOW     560   // 1的低电平：560us
#define BIT_ZERO_HIGH   560   // 0的高电平：560us
#define BIT_ZERO_LOW    1680  // 0的低电平：1680us

// 输入捕获的相关配置
#define IC_CHANNEL       TIM_CHANNEL_2

// 发送状态机状态定义
typedef enum {
    IR_TX_IDLE = 0,         // 空闲状态
    IR_TX_START_PULSE,      // 发送起始脉冲(高电平)
    IR_TX_START_SPACE,      // 发送起始间隔(低电平)
    IR_TX_DATA_HIGH,        // 发送数据位高电平
    IR_TX_DATA_LOW,         // 发送数据位低电平
    IR_TX_STOP,             // 发送结束
} IR_TX_State_t;

// 发送上下文结构体
typedef struct {
    IR_TX_State_t state;        // 当前状态
    uint8_t frame[9];           // 数据帧缓冲区 (8字节数据 + 1字节CRC)
    uint8_t byte_index;         // 当前发送的字节索引
    uint8_t bit_index;          // 当前发送的位索引 (0-7, 7是高位)
    uint8_t total_bytes;        // 总字节数
    bool busy;                  // 发送忙标志
} IR_TX_Context_t;

void IR_Init(void);
bool IR_SendData(uint8_t *data, uint8_t length);
void IR_TX_TimerCallback(TIM_HandleTypeDef *htim);
bool IR_IsTXBusy(void);
void IR_ReceiveData(void);
uint8_t IR_CRC8(uint8_t *data, uint8_t length);
void IR_ResetBuffer(void);

#endif
