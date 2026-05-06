#ifndef DEV_BOARD_COMMUNICATE_H
#define DEV_BOARD_COMMUNICATE_H
#include "FreeRTOS.h"
#include <stdint.h> // 添加以支持 uint32_t

#include "bsp_can.h"
#include "bsp_log.h"

#include "Com_System.h"

#define BOARD_TX_ID_BASE 0x00D
//实际发送的id是 BOARD_TX_ID_BASE + board_id，接收的id是 BOARD_RX_ID_BASE + board_id
#define BOARD_RX_ID_BASE 0x00E


#define UP2DOWN_MESSAGE_TYPE 1
#define DOWN2UP_MESSAGE_TYPE 0
//实验性的宏定义，试试看咋样
// 将实例中的8字节数据缓冲区(data_buffer)的地址转换为指定结构体指针
#define ACCESS_AS_STRUCT(instance, struct_type) ((struct_type *)&(instance)->data_buffer)

#pragma pack(1)
//CAN一帧8字节，所以一共是8*8=64位
/**
 * @brief 下发上数据包结构体
 */
typedef struct{
    uint8_t enable_flag:8; // 1: 使能, 2: 失能
    uint8_t shoot_bool:1;     //0-不射击，1-射击
    uint16_t reserved:7;    // 保留位
    uint16_t up_target:16;   // 上云台目标角度(半精度)
    uint16_t down_yaw_pos:16;// 下云台当前角度(半精度)
    uint16_t up_pitch_target:16;   // 上云台目标俯仰角（半精度）
} down2up_message_t;

/**
 * @brief 上发下数据包结构体
 */
typedef struct{
    uint8_t findbool:1;     // 0-未找到，1-找到
    uint8_t ready_bool:1;   // 0-未就绪，1-上板稳定就绪，下板可开始跟踪
    uint16_t reserved:14;   // 保留位
    uint16_t up_yaw_pos:16;  // 上云台当前角度(半精度)
    uint16_t up_pitch_pos:16;   // 上云台当前俯仰角(半精度)
    uint16_t reserved3:16;   // 保留位
} up2down_message_t;

#pragma pack()

/**
 * @brief 实例，用于板间通信
 * 
 */
typedef struct
{
    uint8_t board_id;                            /**< 板子ID */
    CanInstance_s* can_instance;                /**< CAN实例 */
    uint8_t message_type;                       /**< 消息类型, 决定发送和接收时使用哪种结构体 */

    uint8_t data_buffer[8];                     /**< 用于打包和解包的8字节数据缓冲区 */

    // --- 解码后的数据 (从其他板接收) ---
    uint8_t received_enable_flag;
    uint8_t received_shoot_bool;
    float   received_target_up_yaw;
    float   received_target_up_pitch;
    float   received_current_down_yaw;
    uint8_t received_find_bool;
    uint8_t received_ready_bool;
    float   received_up_yaw_pos;
    float   received_up_pitch_pos;


} board_instance_t;

typedef struct 
{
    uint8_t board_id;
    CanInitConfig_s can_config;
    uint8_t message_type; // 0: down2up_message_t, 1: up2down_message_t
} board_config_t;

typedef union {
    float f;
    uint32_t u;
} FloatUnion;

// 返回指向动态分配内存的指针
board_instance_t* board_init(board_config_t *config);
// 发送函数需要明确传入要发送的数据
void board_send_message(board_instance_t *instance, float data1, float data2, uint8_t flag1, uint8_t flag2);
void Board_Message_Decode(CanInstance_s *can_instance);

#endif