# STM32F1 红外通信模块

基于STM32F1的红外通信系统，支持红外发送/接收、CRC校验、ACK确认机制、CAN总线桥接功能以及智能自发自收过滤。

## 功能特性

- **红外通信**：基于改进的NEC协议（时序为标准NEC的一半）
- **38kHz载波**：使用TIM1 PWM生成38kHz红外载波
- **CRC8校验**：数据传输完整性校验
- **ACK/NACK机制**：双向通信确认
- **超时重传**：提高通信可靠性
- **CAN总线桥接**：CAN与红外数据双向转换
- **模块ID识别**：支持多模块组网
- **⭐ 非阻塞状态机**：主循环永不卡死，支持实时响应
- **⭐ 智能自发自收过滤**：近距离发射/接收隔离，双层冷却期机制
- **⭐ 调试快照系统**：Debug模式下保留原始接收数据，防止被覆盖

## 硬件资源

### 引脚分配

| 功能 | 引脚 | 说明 |
|------|------|------|
| 红外发射 | PA8 | TIM1_CH1 - 38kHz PWM |
| 红外接收 | PA15 | TIM2_CH1 - 输入捕获 |
| CAN RX | PA11 | CAN1接收 |
| CAN TX | PA12 | CAN1发送 |

### 定时器配置

- **TIM1**：38kHz PWM载波生成
- **TIM2**：输入捕获，用于红外接收
- **TIM3**：时序控制，用于红外发送

## 通信协议

### 红外时序

- 起始信号：4.5ms高 + 2.25ms低
- 数据位0：280us高 + 280us低（总计560us）
- 数据位1：280us高 + 840us低（总计1120us）
- 载波频率：38kHz

### 数据帧格式

| 字节0-7 | 字节8 |
|---------|-------|
| 用户数据 | CRC8校验 |

### ACK帧格式

| 字节0 | 字节1 |
|-------|-------|
| 0xA5 | 0xA5 | (ACK) 或 0x5A 0x5A (NACK) |

## 核心特性详解

### 1. 非阻塞状态机设计

主循环采用完全非阻塞设计，避免长时间等待：

```c
// 主循环流程（<1ms完成每次循环）
while (1) {
    IR_CheckRxTimeout();                    // 检查接收超时
    // CAN → 红外（缓存数据）
    // CAN → 红外（非阻塞发送）
    // 红外 → CAN（非阻塞处理）
}
```

**优势**：
- 主循环永不阻塞，响应延迟 < 1ms
- CAN消息实时处理，不会丢失
- 支持双向同时通信

### 2. 智能自发自收过滤

当红外发射管和接收头距离很近时，模块会收到自己发射的信号（回声）。系统采用**双层冷却期机制**智能过滤：

| 帧类型 | 冷却期 | 说明 |
|--------|--------|------|
| 数据帧（8字节） | 80ms | 发送时间长，回声强 |
| ACK/NACK帧（2字节） | 40ms | 发送时间短，需快速响应 |

**配置参数**（`infrared.h`）：
```c
#define IR_RX_SELF_FILTER_MS    80      // 数据帧冷却期
#define IR_ACK_FILTER_MS        40      // ACK帧冷却期（更短）
```

**调整建议**：
- 模块紧贴（<2cm）：增大到 120ms / 60ms
- 近距离（2-5cm）：推荐 80ms / 40ms ✅
- 中距离（5-10cm）：减小到 60ms / 30ms
- 远距离（>10cm）：减小到 40ms / 20ms

### 3. 调试快照系统

解决Debug模式下数据被覆盖无法观察的问题：

**特性**：
- 接收瞬间立即创建数据快照（9字节）
- 循环使用4个槽位，保留最近4次接收记录
- 每个快照包含：数据内容、时间戳、帧类型、有效性标志

**使用方式**：
```c
// 在Debug模式下查看全局变量：
ir_debug_rx_total_count         // 总接收帧数
ir_debug_snapshot[0][0..8]      // 槽位0的9字节数据
ir_debug_snapshot_time[0]       // 接收时刻（ms）
ir_debug_frame_type[0]          // 帧类型：0=数据, 1=ACK, 2=NACK
ir_debug_snapshot_valid[0]      // 有效性标志
```

**辅助函数**：
```c
uint8_t IR_DebugGetLatestSnapshot(void);              // 获取最新快照索引
bool IR_DebugCopySnapshot(uint8_t idx, uint8_t *buf, uint8_t len);  // 复制快照
void IR_DebugClearSnapshots(void);                    // 清除所有快照
const char* IR_DebugGetFrameTypeStr(uint8_t idx);     // 获取帧类型字符串
```

## API接口

### 初始化

```c
void IR_Init(void);
```

### 数据发送

```c
// 发送单帧数据（非阻塞）
bool IR_SendData(uint8_t *data, uint8_t length);

// 带重传发送（阻塞式）
bool IR_SendDataWithRetry(uint8_t *data, uint8_t length, uint8_t max_retry);

// 带ACK确认发送（阻塞式，等待对方ACK）
bool IR_SendDataAndWaitAck(uint8_t *data, uint8_t length, uint8_t max_retry);

// 非阻塞发送ACK/NACK（推荐用于主循环）
bool IR_SendDataAck_NonBlocking(uint8_t status);  // status: 1=ACK, 2=NACK
```

### 接收处理

```c
// 在主循环中检查接收超时
void IR_CheckRxTimeout(void);

// 处理接收到的数据帧（自动发送ACK/NACK）
void IR_ProcessReceivedFrame(uint8_t *data, uint8_t length);

// 检查发送状态
bool IR_IsTXBusy(void);
```

### CRC校验

```c
// 计算CRC8校验值
uint8_t IR_CRC8(uint8_t *data, uint8_t length);
```

### CAN接口

```c
// 发送CAN数据
bool CAN_SendData(uint8_t *data, uint8_t length, uint32_t std_id);
```

## 使用说明

### CAN → 红外

**数据流**：CAN总线 → 本模块 → 红外 → 目标模块

1. 发送CAN帧，ID设为目标模块的 `IR_MODULE_ID`
2. 数据区为8字节用户数据（不含CRC，自动添加）
3. 模块自动通过红外发送（非阻塞）
4. 目标模块收到后回复ACK（可选）

**代码示例**（`main.c`）：
```c
// 第1步：检查CAN数据并缓存
if (can_rx_flag && !IR_IsTXBusy()) {
    if (can_rx_id == IR_MODULE_ID) {
        memcpy(can_to_ir_buffer, can_rx_buffer, 8);
        can_to_ir_ready = 1;
    }
}

// 第2步：发送缓存的CAN数据（非阻塞，自动防自收）
if (can_to_ir_ready && !IR_IsTXBusy()) {
    if (IR_SendData(can_to_ir_buffer, 8)) {
        tx_count++;
        can_to_ir_ready = 0;
    }
}
```

### 红外 → CAN

**数据流**：其他模块 → 红外 → 本模块 → CAN总线

1. 红外接收数据后自动验证CRC
2. 自动创建调试快照（保留原始数据）
3. CRC正确则发送ACK，并通过CAN转发
4. CRC错误则发送NACK，不转发

**代码示例**（`main.c`）：
```c
// 处理红外接收（自动过滤自收数据）
if (ir_rx_complete_flag) {
    ir_rx_complete_flag = 0;

    // 验证CRC（第9字节是CRC）
    uint8_t calculated_crc = IR_CRC8(received_data, 8);
    if (calculated_crc == received_data[8]) {
        // CRC正确，发送ACK（非阻塞）
        IR_SendDataAck_NonBlocking(1);
        // 转发到CAN总线
        CAN_SendData(received_data, 8, IR_MODULE_ID);
    } else {
        // CRC错误，发送NACK
        IR_SendDataAck_NonBlocking(2);
    }
}
```

### 模块配置

在 `infrared.h` 中修改：

```c
#define IR_MODULE_ID            0x01   // 修改为唯一模块ID (1-255)

// 自发自收过滤参数调整
#define IR_RX_SELF_FILTER_MS    80      // 数据帧冷却期
#define IR_ACK_FILTER_MS        40      // ACK帧冷却期

// 超时参数
#define IR_RX_TIMEOUT_MS        50      // 接收超时时间
#define IR_TX_FRAME_INTERVAL_MS 30      // 帧间隔
#define IR_ACK_TIMEOUT_MS       100     // 等待ACK超时
```

## 调试指南

### 使用调试快照系统

**步骤1**：启动Debug模式，在Watch窗口添加变量：
```
ir_debug_rx_total_count
ir_debug_snapshot[0][0] ~ ir_debug_snapshot[0][8]
ir_debug_frame_type[0]
ir_debug_snapshot_valid[0]
```

**步骤2**：运行程序并暂停，检查：
- `ir_debug_rx_total_count` > 0 表示有数据接收
- `ir_debug_frame_type[0]` = 0（数据帧）、1（ACK）、2（NACK）
- `ir_debug_snapshot_valid[0]` = 1 表示数据有效

**步骤3**：找到最新快照：
```c
// 最新快照索引 = (当前索引 - 1 + 4) % 4
uint8_t latest = (ir_debug_snapshot_index - 1 + 4) % 4;
// 查看 ir_debug_snapshot[latest][0..8]
```

### 常见问题排查

| 问题现象 | 可能原因 | 解决方案 |
|----------|----------|----------|
| 收不到数据 | 硬件连接问题 | 检查PA8/PA15接线 |
| 收到自己的回声 | 冷却期太短 | 增大 `IR_RX_SELF_FILTER_MS` |
| 漏掉对方数据 | 冷却期太长 | 减小 `IR_RX_SELF_FILTER_MS` |
| CRC校验失败 | 信号干扰 | 检查红外头对准情况 |
| Debug看不到数据 | 数据被覆盖 | 使用 `ir_debug_snapshot` 查看 |

### 性能监控变量

以下变量可用于实时监控通信状态：

```c
// 发送统计
volatile uint8_t tx_count;              // 成功发送计数
volatile uint32_t ir_last_tx_complete_time;  // 最后发送完成时间

// 接收统计
volatile uint16_t ir_debug_rx_total_count;   // 总接收帧数
volatile uint8_t ir_rx_complete_flag;        // 数据帧接收标志
volatile uint8_t ir_ack_received_flag;       // ACK接收标志

// 自发自收状态
volatile uint8_t ir_rx_ignore_self;          // 是否处于冷却期
volatile uint32_t ir_last_ack_tx_time;       // 最后ACK发送时间
```

## 编译环境

- STM32CubeMX
- Keil MDK-ARM / STM32CubeIDE
- STM32F1xx HAL库
- ARM Compiler 5/6

## 文件结构

```
Core/
├── Inc/
│   └── infrared.h          # 红外模块头文件（配置参数）
├── Src/
│   ├── main.c              # 主程序（非阻塞状态机）
│   ├── infrared.c          # 红外模块实现
│   └── can.c               # CAN接口实现
└── Startup/
    └── startup_stm32f103xb.s
```

## 注意事项

- **模块ID唯一性**：同一网络中每个模块的 `IR_MODULE_ID` 必须不同
- **时序参数**：两个相同模块通信时，时序参数不宜过低（已优化为高速模式）
- **红外对准**：红外发射/接收头需对准，避免遮挡和强光干扰
- **距离调整**：根据模块间距调整 `IR_RX_SELF_FILTER_MS` 参数
- **Debug模式**：单步执行时使用 `ir_debug_snapshot` 查看接收数据

## 版本历史

### v2.0 (最新)
- ✅ 添加非阻塞状态机设计
- ✅ 添加智能自发自收过滤（双层冷却期）
- ✅ 添加调试快照系统
- ✅ 优化主循环响应速度
- ✅ 添加完整的调试辅助函数

### v1.0
- 基础红外通信功能
- CRC校验和ACK机制
- CAN总线桥接

---

**注意**：以上为项目文档，持续更新中。如有问题请参考代码注释或联系开发者。
