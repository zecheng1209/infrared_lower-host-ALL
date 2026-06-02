/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "infrared.h"
#include "can.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
// 外部声明CAN接收环形队列
#define CAN_RX_QUEUE_SIZE  4
typedef struct {
    uint32_t can_id;
    uint8_t  data[8];
    uint8_t  dlc;
} CanRxSlot_t;

extern volatile CanRxSlot_t can_rx_queue[CAN_RX_QUEUE_SIZE];
extern volatile uint8_t can_rx_head;
extern volatile uint8_t can_rx_tail;
extern volatile uint8_t can_rx_count;

// ===== 运行时模块ID（可通过CMD_SET_ID远程配置） =====
volatile uint8_t my_module_id = IR_MODULE_ID;  // 默认取infrared.h中的值

// ===== CAN→红外 发送队列（FIFO，防丢帧） =====
#define CAN_TO_IR_QUEUE_SIZE  4
typedef struct {
    uint8_t data[8];
    uint8_t valid;
} CanToIrSlot_t;
CanToIrSlot_t can_to_ir_queue[CAN_TO_IR_QUEUE_SIZE];
volatile uint8_t can_to_ir_head = 0;
volatile uint8_t can_to_ir_tail = 0;
volatile uint8_t can_to_ir_count = 0;

// ===== 红外→CAN 转发队列（FIFO，防丢帧） =====
#define IR_TO_CAN_QUEUE_SIZE   4
typedef struct {
    uint32_t can_id;
    uint8_t  data[8];
    uint8_t  dlc;
    uint8_t  valid;
} IrToCanSlot_t;
IrToCanSlot_t ir_to_can_queue[IR_TO_CAN_QUEUE_SIZE];
volatile uint8_t ir_to_can_head = 0;
volatile uint8_t ir_to_can_tail = 0;
volatile uint8_t ir_to_can_count = 0;

// ===== CAN多模块错峰参数 =====
// 按模块ID低位顺序延迟响应，避免多模块同时抢占总线(hal_delay)
// 0x10→0ms, 0x11→3ms, 0x12→6ms ...
#define IR_CAN_STAGGER_BASE_MS  3

// ===== 调试计数器 =====
volatile uint32_t dbg_can_cmd_received = 0;
volatile uint32_t dbg_can_data_received = 0;
volatile uint32_t dbg_ir_tx_from_can = 0;
volatile uint32_t dbg_ir_rx_data = 0;
volatile uint32_t dbg_ir_rx_ack = 0;
volatile uint32_t dbg_can_data_sent = 0;
volatile uint32_t dbg_can_ack_sent = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM1_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  IR_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
// ===== 下位机 main loop =====
// 协议说明（CAN ID编码方案C）：
//   CAN ID = (type<<8) | module_id,  type: 1=CMD / 2=DATA / 3=ACK
//   CMD帧  ID=0x1[module_id]: [模块ID(1B)][命令码(1B)][数据×6] → 下位机回复ACK
//   DATA帧 ID=0x2[module_id]: [数据×8] 纯数据              → 下位机转发到红外(9B含CRC)
//   ACK帧  ID=0x3[module_id]: [0xA5][0xA5]                → 上位机确认收到
//   IR数据帧: [8B数据][1B CRC] = 9B(72bit)               → 下位机转发到CAN

while (1)
{
    IR_CheckRxTimeout();

    // ====================================================================
    //  第1步: CAN接收 → 处理/入队（不阻塞，不丢帧）
    // ====================================================================
    if (can_rx_count > 0)
    {
        volatile CanRxSlot_t *slot = &can_rx_queue[can_rx_tail];

        if (IR_HOST_IS_CMD_FRAME(slot->can_id) && slot->dlc >= 2)
        {
            uint8_t rx_module_id = IR_HOST_CAN_ID_GET_MODULE(slot->can_id);

            // 匹配自身ID 或 广播ID(0x00) 都响应
            if (rx_module_id == my_module_id || rx_module_id == 0x00)
            {
                // ★ 错峰：按模块ID延迟响应，避免多模块同时抢占总线
                HAL_Delay((my_module_id & 0x0F) * IR_CAN_STAGGER_BASE_MS);

                uint8_t cmd = slot->data[1];
                dbg_can_cmd_received++;

                if (cmd == IR_HOST_CMD_PING || cmd == IR_HOST_CMD_READ_STATUS)
                {
                    CAN_SendFrame(IR_HOST_CAN_ID_ACK(my_module_id),
                                  (uint8_t[]){IR_ACK_MAGIC, IR_ACK_MAGIC}, 2);
                    dbg_can_ack_sent++;
                }
                else if (cmd == IR_HOST_CMD_RESET)
                {
                    IR_ResetBuffer();
                    CAN_SendFrame(IR_HOST_CAN_ID_ACK(my_module_id),
                                  (uint8_t[]){IR_ACK_MAGIC, IR_ACK_MAGIC}, 2);
                    dbg_can_ack_sent++;
                }
                else if (cmd == IR_HOST_CMD_SET_ID && slot->dlc >= 3)
                {
                    uint8_t new_id = slot->data[2];
                    if (new_id >= 0x01 && new_id <= 0xFE)
                    {
                        my_module_id = new_id;
                    }
                    CAN_SendFrame(IR_HOST_CAN_ID_ACK(my_module_id),
                                  (uint8_t[]){IR_ACK_MAGIC, IR_ACK_MAGIC}, 2);
                    dbg_can_ack_sent++;
                }
            }
        }
        else if (IR_HOST_IS_DATA_FRAME(slot->can_id) && slot->dlc >= 1)
        {
            uint8_t rx_module_id = IR_HOST_CAN_ID_GET_MODULE(slot->can_id);

            // DATA帧 → 回CAN ACK + 入队等红外空闲时转发
            if (rx_module_id == my_module_id && can_to_ir_count < CAN_TO_IR_QUEUE_SIZE)
            {
                // ★ 错峰：按模块ID延迟响应ACK
                HAL_Delay((my_module_id & 0x0F) * IR_CAN_STAGGER_BASE_MS);

                // 先回CAN ACK，告诉上位机已收到
                CAN_SendFrame(IR_HOST_CAN_ID_ACK(my_module_id),
                              (uint8_t[]){IR_ACK_MAGIC, IR_ACK_MAGIC}, 2);
                dbg_can_ack_sent++;

                CanToIrSlot_t *ir_slot = &can_to_ir_queue[can_to_ir_head];
                memset(ir_slot->data, 0, 8);
                memcpy(ir_slot->data, (const void *)slot->data, slot->dlc > 8 ? 8 : slot->dlc);
                ir_slot->valid = 1;
                can_to_ir_head = (can_to_ir_head + 1) % CAN_TO_IR_QUEUE_SIZE;
                can_to_ir_count++;
                dbg_can_data_received++;
            }
        }

        // 出队（关中断保护，防止ISR的count++被覆盖）
        __disable_irq();
        can_rx_tail = (can_rx_tail + 1) % CAN_RX_QUEUE_SIZE;
        can_rx_count--;
        __enable_irq();
    }

    // ====================================================================
    //  第2步: CAN→红外 出队发送（红外空闲时才发）
    // ====================================================================
    if (can_to_ir_count > 0 && !IR_IsTXBusy())
    {
        CanToIrSlot_t *slot = &can_to_ir_queue[can_to_ir_tail];
        if (slot->valid)
        {
            if (IR_SendData(slot->data, 8))
            {
                slot->valid = 0;
                can_to_ir_tail = (can_to_ir_tail + 1) % CAN_TO_IR_QUEUE_SIZE;
                can_to_ir_count--;
                dbg_ir_tx_from_can++;
            }
        }
    }

    // ====================================================================
    //  第3步: 红外接收 → 入队（不阻塞，不丢帧）
    // ====================================================================

    // 红外收到数据帧 → 回红外ACK + 数据入队等CAN转发
    if (ir_rx_complete_flag)
    {
        ir_rx_complete_flag = 0;
        dbg_ir_rx_data++;

        uint8_t calculated_crc = IR_CRC8(received_data, 8);
        if (calculated_crc == received_data[8])
        {
            IR_SendDataAck_NonBlocking(1);

            if (ir_to_can_count < IR_TO_CAN_QUEUE_SIZE)
            {
                IrToCanSlot_t *slot = &ir_to_can_queue[ir_to_can_head];
                slot->can_id = IR_HOST_CAN_ID_DATA(my_module_id);
                memcpy(slot->data, received_data, 8);
                slot->dlc = 8;
                slot->valid = 1;
                ir_to_can_head = (ir_to_can_head + 1) % IR_TO_CAN_QUEUE_SIZE;
                ir_to_can_count++;
            }
        }
        else
        {
            IR_SendDataAck_NonBlocking(2);
        }
    }

    // 红外收到ACK/NACK → 入队等CAN转发
    if (ir_ack_received_flag)
    {
        ir_ack_received_flag = 0;
        dbg_ir_rx_ack++;

        if (ir_to_can_count < IR_TO_CAN_QUEUE_SIZE)
        {
            IrToCanSlot_t *slot = &ir_to_can_queue[ir_to_can_head];
            slot->can_id = IR_HOST_CAN_ID_ACK(my_module_id);
            if (ir_ack_status == 1)
            {
                uint8_t ack_data[2] = {IR_ACK_MAGIC, IR_ACK_MAGIC};
                memcpy(slot->data, ack_data, 2);
                slot->dlc = 2;
            }
            else
            {
                uint8_t nack_data[2] = {IR_NACK_MAGIC, IR_NACK_MAGIC};
                memcpy(slot->data, nack_data, 2);
                slot->dlc = 2;
            }
            slot->valid = 1;
            ir_to_can_head = (ir_to_can_head + 1) % IR_TO_CAN_QUEUE_SIZE;
            ir_to_can_count++;
        }
    }

    // ====================================================================
    //  第4步: 红外→CAN 出队转发
    // ====================================================================
    if (ir_to_can_count > 0)
    {
        IrToCanSlot_t *slot = &ir_to_can_queue[ir_to_can_tail];
        if (slot->valid)
        {
            // ★ 错峰：数据转发也按模块ID延迟，避免多模块同时发送DATA帧
            HAL_Delay((my_module_id & 0x0F) * IR_CAN_STAGGER_BASE_MS);
            CAN_SendFrame(slot->can_id, slot->data, slot->dlc);
            slot->valid = 0;
            ir_to_can_tail = (ir_to_can_tail + 1) % IR_TO_CAN_QUEUE_SIZE;
            ir_to_can_count--;
            if (IR_HOST_IS_DATA_FRAME(slot->can_id)) dbg_can_data_sent++;
            else dbg_can_ack_sent++;
        }
    }
}
/* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
