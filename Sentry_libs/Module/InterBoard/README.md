# InterBoard 板间通信模块

本模块用于在双板架构（如上板/下板、云台/底盘）之间进行解耦和自动推断身份的高效数据同步。目前已内置针对 CAN 协议的快速接入胶水层，同时也支持开发者随时接入如 UART、SPI 等其他任意的硬件抽象。

## 模块特色
- **身份自动推断**：取消了旧版需要手动指明打包格式（Message Type = 0 还是 1）的屎山代码。在初始化时仅需传入 `ID_DOWN` 或 `ID_UP`，模块内依靠 ID 字段全自动判断您是哪块板，发送与接收数据时的解包格式会进行自动匹配。
- **协议解耦与抽象 (Hardware Abstraction)**：模块核心逻辑与底层的通讯协议解耦。发送被抽象为一个函数指针回调，上下文使用泛型无类型指针承接。这意味着您可以将这套统一逻辑瞬间迁移到任意类型的其他通信层上。
- **数据面与解包逻辑隔离**：去除了将 CAN 返回的数组进行晦涩强转的现象，在模块内部通过标准的 `pack(1)` 强制对齐结构体进行解包，并将数据**统一更新暴露给应用层结构体** (`InterBoard_AppStruct_t app_data`)，读写极为直观。

---

## 快速使用说明 (使用内置的CAN层)

大部分情境下，若硬件依然沿用 CAN，您可直接使用专门准备的快速入口进行初始化。

### 1. 初始化
在您的通信任务或应用层初始化代码中：
```c
#include "inter_board.h"

// 创建一个全局句柄
InterBoard_Instance_t* g_inter_board;

void MyTask_Init(void) {
    // 您只需要传入您这块板的角色 ID (INTER_BOARD_ID_DOWN 或 INTER_BOARD_ID_UP)
    // 所有的 CAN ID 映射 (0x00E / 0x00D) 以及收发中断回调逻辑会自动绑定完成！
    g_inter_board = InterBoard_Init_CAN(1, INTER_BOARD_ID_DOWN); // 第一个参数是您使用的CAN编号(如1或2), 第二个参数是您的板子身份 
}
```

### 2. 发送数据
无论是上板发送给下板，还是下板发给上板，只需对 `app_data` 内针对您的板子的 `tx_xxx` 字段赋值，并调用一个没有任何参数重压的发送函数：

```c
void MyTask_Loop(void) {
    // 1. 将您要发送的数据赋值（假设本板为下板 DOWN，负责发 target_up_yaw 等）：
    g_inter_board->app_data.tx_control_mode = 1;
    g_inter_board->app_data.tx_shoot_bool = 0;
    g_inter_board->app_data.tx_target_up_yaw = 30.5f;

    // 2. 调用发送：它会根据本板的身份，自动把相关的 tx 数据组装打包并丢到 CAN 上发送
    InterBoard_Send(g_inter_board);
}
```

### 3. 读取（接收）数据
得益于我们在初始化时的自动绑定，当有 CAN 数据从对面板子推上来后，模块底层会自动解码出相应的浮点数和状态并覆盖到 `app_data.rx_...`。
我们在业务循环中直接读取即可：

```c
void Print_PeerData(void) {
    // 只要有数据过来，后台中断一直刷入这些字段，随意读取
    float peer_yaw = g_inter_board->app_data.rx_up_yaw_pos;
    uint8_t target_found = g_inter_board->app_data.rx_find_bool;
    
    // ...将其派发给PID或者状态机
}
```

---

## 高阶用法：如何移植到 UART（串口）等其他协议上

得益于良好的抽象接口，想要将通信媒介无缝从 CAN 切到 UART，或使用其他通信手段，**您只需两步：**

1. 自己写一个 UART 底层的配置实例化。
2. 包装并挂载一个发送和回调的胶水。

**代码示例：**

```c
/* 第一步：实现您自定义通信的 发送胶水函数 */
void MyCustomUART_TxCallback(InterBoard_Instance_t* instance, uint8_t* data, uint16_t len) {
    // 获取您的串口句柄，从模块传回了纯净的 8Bytes Payload，您直接发送出去即可
    UartInstance_s *uart = (UartInstance_s *)instance->transport_ctx;
    HAL_UART_Transmit(uart->huart, data, len, 100);
}

/* 您的应用初始化 */
void MyCustomComm_Init(void) {
    // 分配基础模块的通信环境
    InterBoard_Config_t config = {0};
    config.board_id = INTER_BOARD_ID_UP; // 我是上板
    config.tx_callback = MyCustomUART_TxCallback; // 挂载发送回调接口
    
    // 此处假设您已经有了 UART_Init 并拿到了一个自定义的句柄，把它作为通用上下文赛给模块
    config.transport_ctx = my_uart_handle; 

    // 初始化核心无关逻辑实例
    g_inter_board = InterBoard_Init(&config);
}

/* 第二步：当您的串口中断接受到了8字节后，丢给核心模块解包 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart == my_uart_handle->huart) {
        // rx_buffer 即为您从串口收到的 8字节 对侧数据
        InterBoard_ReceivePayload(g_inter_board, rx_buffer, 8);
    }
}
```
经过以上步骤，您的结构便能够通过任何总线完美继承全部的自动身份推断解包机制，不需要重新写或者触碰到包含 `FloatToHalf` 之类的繁琐逻辑。
