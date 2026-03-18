#ifndef INTER_BOARD_H
#define INTER_BOARD_H

#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include "bsp_can.h"
#include "bsp_log.h"
#include "Com_System.h"

/* 板子ID约定：根据ID自动推断是上板还是下板 */
#define INTER_BOARD_ID_DOWN 0
#define INTER_BOARD_ID_UP   1

/* 角色定义 (内部根据ID推演，对外透明) */
typedef enum {
    INTER_BOARD_ROLE_DOWN = 0, // 下板：发送Down2Up，接收Up2Down
    INTER_BOARD_ROLE_UP   = 1  // 上板：发送Up2Down，接收Down2Up
} InterBoard_Role_e;

#pragma pack(1)

/* 下发上数据包 (8 Bytes) */
typedef struct {
    uint8_t control_mode : 8;
    uint8_t shoot_bool : 1;
    uint16_t reserved : 7;
    uint16_t up_target;
    uint16_t down_yaw_pos;
    uint16_t up_pitch_target;
} InterBoard_Down2UpMsg_t;

/* 上发下数据包 (8 Bytes) */
typedef struct {
    uint8_t findbool : 1;
    uint16_t reserved : 15;
    uint16_t up_yaw_pos;
    uint16_t up_pitch_pos;
    uint16_t reserved3;
} InterBoard_Up2DownMsg_t;

#pragma pack()

/* 提供给外部操作的应用层数据结构体：不包含底层打包逻辑，完全解析后的浮点和状态信息 */
typedef struct {
    /* 接收端最新数据 */
    uint8_t rx_control_mode;
    uint8_t rx_shoot_bool;
    float   rx_target_up_yaw;
    float   rx_target_up_pitch;
    float   rx_current_down_yaw;
    
    uint8_t rx_find_bool;
    float   rx_up_yaw_pos;
    float   rx_up_pitch_pos;

    /* 发送端最新数据缓存(可由用户直接赋值，在此统一管理当前要发送的状态) */
    uint8_t tx_control_mode;
    uint8_t tx_shoot_bool;
    float   tx_target_up_yaw;
    float   tx_target_up_pitch;
    float   tx_current_down_yaw;
    
    uint8_t tx_find_bool;
    float   tx_up_yaw_pos;
    float   tx_up_pitch_pos;
} InterBoard_AppStruct_t;

struct InterBoard_Instance_t;

/* 抽象通信接口：发送回调函数 */
typedef void (*InterBoard_TxCallback)(struct InterBoard_Instance_t* instance, uint8_t* data, uint16_t len);

/* 板间通信实例结构体 */
typedef struct InterBoard_Instance_t {
    uint8_t board_id;                   /* 板子ID */
    InterBoard_Role_e role;             /* 根据ID自动确定的角色 */
    
    InterBoard_AppStruct_t app_data;    /* 应用层使用的数据集合 */
    uint8_t tx_buffer[8];               /* 发送序列化缓存 */
    uint8_t rx_buffer[8];               /* 接收序列化缓存 */
    
    /* 抽象通信层 */
    InterBoard_TxCallback tx_callback;  /* 底层发送接口 */
    void* transport_ctx;                /* 底层通信实例指针(如 CanInstance_s*, UartInstance_s* 等) */
} InterBoard_Instance_t;

/* 初始化配置结构体 (纯抽象逻辑使用) */
typedef struct {
    uint8_t board_id;
    InterBoard_TxCallback tx_callback;
    void* transport_ctx;
} InterBoard_Config_t;

/* ========================================================== */
/* 核心逻辑抽象 APIs */     
/* ========================================================== */

/**
 * @brief 初始化板间通信逻辑实例（与具体通信方式解耦）
 */
InterBoard_Instance_t* InterBoard_Init(InterBoard_Config_t *config);

/**
 * @brief 发送一帧报文，提取 app_data 生成 payload，并调用 tx_callback 抽象发送
 */
void InterBoard_Send(InterBoard_Instance_t *instance);

/**
 * @brief 通用接收入口：任何底层接口收到数据后只需调用此函数，传入 payload 即可
 */
void InterBoard_ReceivePayload(InterBoard_Instance_t *instance, uint8_t* data, uint16_t len);

/**
 * @brief 暴露可单独使用的半精度浮点转换工具
 */
uint16_t InterBoard_FloatToHalf(float rawdata);
float InterBoard_HalfToFloat(uint16_t halfdata);

/* ========================================================== */
/* CAN通信适配层 APIs (为了兼容当前工程的快速接入) */     
/* ========================================================== */

#define INTER_BOARD_TX_ID_BASE 0x00E
#define INTER_BOARD_RX_ID_BASE 0x00D

/**
 * @brief 使用 CAN 作为底层通信初始化板间通信
 * @param board_id  自身板号
 * @return 创建好的全家桶实例
 */
InterBoard_Instance_t* InterBoard_Init_CAN(uint8_t can_number, uint8_t board_id);

/**
 * @brief 暴露给 CAN 底层的回调解码(无需用户调用，初始化会自动绑定)
 */
void InterBoard_CAN_DecodeCallback(CanInstance_s *can_instance);

#endif // INTER_BOARD_H