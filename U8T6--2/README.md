# 红外通信下位机模块

## 项目简介

这是一个基于STM32F103微控制器的红外通信下位机模块，负责实现CAN总线与红外通信之间的数据转发。该模块支持38kHz红外载波通信，具备CRC校验、ACK确认机制和超时重传功能，用于与上位机进行无线数据传输。

## 硬件需求

- STM32F103C8T6开发板
- 红外发射/接收模块（支持38kHz载波）
- CAN总线接口
- 电源系统

## 软件需求

- Keil MDK-ARM开发环境
- STM32CubeMX
- STM32F1xx HAL库

## 项目结构

```
U8T6--2/
├── Core/                  # 核心代码
│   ├── Inc/               # 头文件
│   │   ├── can.h
│   │   ├── gpio.h
│   │   ├── main.h
│   │   └── stm32f1xx_hal_conf.h
│   └── Src/               # 源代码
│       ├── infrared.c     # 红外通信核心实现
│       ├── infrared.h     # 红外通信头文件
│       ├── can.c          # CAN总线驱动
│       ├── main.c         # 主程序入口
│       └── tim.c          # 定时器配置
└── Drivers/               # 驱动文件
    └── CMSIS/             # CMSIS核心库
```

## 核心功能

### 1. 红外通信协议
- 38kHz载波调制
- 自定义通信帧格式（72位 = 8字节数据 + 1字节CRC）
- 支持数据帧、ACK帧、NACK帧

### 2. CAN总线通信
- CAN总线数据接收和发送
- 帧ID编码方案：`CAN ID = (type<<8) | module_id`
- 支持CMD帧(0x1xx)、DATA帧(0x2xx)、ACK帧(0x3xx)

### 3. 数据转发
- CAN → 红外：将CAN数据帧转发到红外链路
- 红外 → CAN：将红外数据帧转发到CAN总线（可配置为单向/双向模式）

### 4. 可靠性机制
- CRC8校验
- ACK/NACK确认机制
- 超时重传支持
- 自发自收过滤机制

## 通信协议说明

### CAN ID编码方案（方案C）

| 帧类型 | ID格式 | 说明 |
|--------|--------|------|
| CMD帧 | 0x1[module_id] | 命令帧：[模块ID(1B)][命令码(1B)][数据×6] |
| DATA帧 | 0x2[module_id] | 数据帧：[数据×8]，纯数据转发 |
| ACK帧 | 0x3[module_id] | 确认帧：[0xA5][0xA5] |

### 红外数据帧格式

```
[8字节数据][1字节CRC] = 9字节(72位)
```

### 支持的命令

| 命令码 | 命令名称 | 说明 |
|--------|----------|------|
| IR_HOST_CMD_PING | PING | 心跳检测 |
| IR_HOST_CMD_READ_STATUS | 读取状态 | 获取模块状态 |
| IR_HOST_CMD_RESET | 重置 | 重置缓冲区 |

## 核心API

### 红外通信API

#### `IR_Init()`
初始化红外通信模块
- **参数**：无
- **返回值**：无

#### `IR_SendData(uint8_t *data, uint8_t length)`
发送红外数据（非阻塞）
- **参数**：数据缓冲区、数据长度(1-8字节)
- **返回值**：成功返回true，失败返回false

#### `IR_IsTXBusy()`
检查发送是否忙
- **参数**：无
- **返回值**：忙返回true，空闲返回false

#### `IR_SendDataAndWaitAck(uint8_t *data, uint8_t length, uint8_t max_retry)`
带ACK确认的数据发送
- **参数**：数据缓冲区、数据长度、最大重传次数
- **返回值**：成功返回true，失败返回false

### CAN通信API

#### `CAN_SendFrame(uint32_t can_id, uint8_t *data, uint8_t dlc)`
发送CAN帧
- **参数**：CAN ID、数据缓冲区、数据长度
- **返回值**：无

## 使用说明

### 1. 硬件连接
- 将红外发射模块连接到PA8（TIM1_CH1，38kHz PWM输出）
- 将红外接收模块连接到PA15（TIM2_CH1，输入捕获）
- 连接CAN总线到CAN_RX和CAN_TX引脚
- 确保电源供应稳定

### 2. 软件配置
- 使用STM32CubeMX生成基础代码
- 配置定时器：TIM1(38kHz PWM)、TIM2(输入捕获)、TIM3(发送时序控制)
- 配置CAN总线参数

### 3. 编译和烧录
- 使用Keil MDK-ARM编译项目
- 通过JTAG或串口烧录到STM32开发板

### 4. 运行模式
- 默认工作在单向测试模式（仅CAN→红外转发）
- 如需双向通信，取消main.c中相关代码的注释

## 代码架构

### 核心模块

#### 1. 红外通信模块 (`infrared.c`)
- 实现38kHz载波调制和解调
- 处理帧接收和发送
- 实现CRC校验和ACK机制

#### 2. CAN总线模块 (`can.c`)
- CAN总线初始化和配置
- 数据帧的发送和接收

#### 3. 主程序模块 (`main.c`)
- 系统初始化和主循环
- CAN与红外的数据转发逻辑
- 协议解析和命令处理

## 调试功能

### 数据快照系统
- 支持保存最近4次接收数据的快照
- 记录帧类型（DATA/ACK/NACK）和时间戳
- 提供调试辅助函数：
  - `IR_DebugGetLatestSnapshot()` - 获取最新快照索引
  - `IR_DebugCopySnapshot()` - 复制快照数据
  - `IR_DebugClearSnapshots()` - 清除所有快照
  - `IR_DebugGetFrameTypeStr()` - 获取帧类型描述

## 注意事项

1. **红外模块配置**：确保红外发射/接收模块支持38kHz载波
2. **CAN总线配置**：确保CAN总线的波特率和过滤配置正确
3. **电源供应**：确保模块有稳定的电源供应
4. **通信距离**：红外通信距离有限，注意安装位置
5. **调试模式**：建议在调试时启用数据快照功能

## 许可证

本项目基于STMicroelectronics的HAL库，遵循ST的许可证条款。

---

*项目版本：1.0.0*
*最后更新：2026-05-07*