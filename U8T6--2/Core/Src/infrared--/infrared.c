#include "infrared.h"
#include "stm32f1xx_hal.h"
#include <string.h>

extern TIM_HandleTypeDef htim1;  // 用于 38KHz PWM 载波 (PA8 - TIM1_CH1)
extern TIM_HandleTypeDef htim2;  // 用于接收输入捕获 (PA15 - TIM2_CH1)
extern TIM_HandleTypeDef htim3;  // 用于发送时序控制

uint8_t received_data[9];
uint8_t bit_index = 0;
uint32_t last_capture_time = 0;
uint8_t receiving = 0;
uint32_t rx_last_activity_time = 0;

static IR_TX_Context_t tx_context = {0};

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

    tx_context.state = IR_TX_START_PULSE;
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    
    IR_TX_SetNextTimer(START_PULSE_LEN);
    HAL_TIM_Base_Start_IT(&htim3);

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

        case IR_TX_STOP_PULSE:
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_RESET);
            tx_context.state = IR_TX_IDLE;
            tx_context.busy = false;
            tx_context.last_tx_time = HAL_GetTick();
            HAL_TIM_Base_Stop_IT(htim);
            break;

        default:
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_GPIO_WritePin(IR_TX_GPIO_PORT, IR_TX_GPIO_PIN, GPIO_PIN_RESET);
            tx_context.busy = false;
            tx_context.state = IR_TX_IDLE;
            HAL_TIM_Base_Stop_IT(htim);
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
            if (pulse_duration >= (RX_START_THRESHOLD - IR_PULSE_TOLERANCE_US) && 
                pulse_duration <= (RX_START_THRESHOLD + IR_PULSE_TOLERANCE_US)) {
                receiving = 1;
                bit_index = 0;
                memset(received_data, 0, 9);
            }
        } else {
            if (bit_index >= 72) {
                last_capture_time = capture_value;
                return;
            }
            
            if (pulse_duration >= (RX_BIT0_THRESHOLD - IR_PULSE_TOLERANCE_US) && 
                pulse_duration <= (RX_BIT0_THRESHOLD + IR_PULSE_TOLERANCE_US)) {
                bit_index++;
            }
            else if (pulse_duration >= (RX_BIT1_THRESHOLD - IR_PULSE_TOLERANCE_US) && 
                     pulse_duration <= (RX_BIT1_THRESHOLD + IR_PULSE_TOLERANCE_US)) {
                received_data[bit_index / 8] |= (1 << (7 - (bit_index % 8)));
                bit_index++;
            }
            else if (pulse_duration > IR_MAX_PULSE_US) {
                receiving = 0;
                bit_index = 0;
                last_capture_time = capture_value;
                return;
            }
            
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

void IR_ReceiveData(void) {
    // 数据已接收完成，CRC校验在应用层进行
    // 此函数保留用于后续扩展数据处理逻辑
}

void IR_ResetBuffer(void)
{
    memset(received_data, 0, 9);
    bit_index = 0;
    receiving = 0;
}

bool IR_SendDataWithRetry(uint8_t *data, uint8_t length, uint8_t max_retry)
{
    if (length > 8) return false;
    
    for (uint8_t retry = 0; retry < max_retry; retry++) {
        while (IR_IsTXBusy()) {
            HAL_Delay(1);
        }
        
        if (IR_SendData(data, length)) {
            while (IR_IsTXBusy()) {
                HAL_Delay(1);
            }
            HAL_Delay(10);
            return true;
        }
        
        HAL_Delay(5);
    }
    
    return false;
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
