#include "inter_board.h"
#include <string.h>

typedef union {
    float f;
    uint32_t u;
} FloatUnion;

/**
 * @brief Float转Half (浮点数转半精度)
 */
uint16_t InterBoard_FloatToHalf(float rawdata) {
    uint16_t result;
    FloatUnion fu;
    fu.f = rawdata; 
    uint32_t f32_bits = fu.u;

    uint32_t sign_f32 = (f32_bits >> 31) & 0x01;
    uint32_t exp_f32 = (f32_bits >> 23) & 0xFF;
    uint32_t mant_f32 = f32_bits & 0x7FFFFF;

    if(exp_f32 == 0) return (sign_f32 << 15);
    if(exp_f32 == 255) return (sign_f32 << 15) | 0x7C00 | (mant_f32 >> 13);
    if(exp_f32 < 113) return (sign_f32 << 15);
    if(exp_f32 > 142) return (sign_f32 << 15) | 0x7C00;
    
    return (uint16_t)(sign_f32 << 15) | ((exp_f32 - 112) << 10) | (mant_f32 >> 13);
}

/**
 * @brief Half转Float (半精度转浮点)
 */
float InterBoard_HalfToFloat(uint16_t halfdata) {
    float result;
    uint32_t sign = (halfdata >> 15) & 0x1;
    uint32_t exp = (halfdata >> 10) & 0x1F;
    uint32_t mant = halfdata & 0x3FF;
    
    uint32_t floatdata;
    if(exp == 0 && mant == 0) floatdata = (sign << 31);
    else if(exp == 0x1F) floatdata = (sign << 31) | (0xFF << 23) | (mant << 13);
    else floatdata = (sign << 31) | ((exp - 15 + 127) << 23) | (mant << 13);
    
    FloatUnion fu;
    fu.u = floatdata;
    return fu.f;
}

/* ========================================================== */
/* 核心逻辑抽象层实现 */     
/* ========================================================== */

/**
 * @brief 核心初始化：解耦通信与逻辑
 */
InterBoard_Instance_t* InterBoard_Init(InterBoard_Config_t *config) {
    if(config == NULL) return NULL;

    InterBoard_Instance_t* instance = (InterBoard_Instance_t*)pvPortMalloc(sizeof(InterBoard_Instance_t));
    if (instance == NULL) return NULL;
    memset(instance, 0, sizeof(InterBoard_Instance_t));
  
    instance->board_id = config->board_id;
    instance->tx_callback = config->tx_callback;
    instance->transport_ctx = config->transport_ctx;

    // 自动推断角色身份
    if(instance->board_id == INTER_BOARD_ID_DOWN) {
        instance->role = INTER_BOARD_ROLE_DOWN;
    } else {
        instance->role = INTER_BOARD_ROLE_UP; // 默认或者为 INTER_BOARD_ID_UP
    }

    return instance;
}

/**
 * @brief 核心发送：仅打包数据并丢给回调
 */
void InterBoard_Send(InterBoard_Instance_t *instance) {
    if(instance == NULL || instance->tx_callback == NULL) return;

    memset(instance->tx_buffer, 0, 8); // 清除老数据

    // 根据自身角色，选择性打包对于的发送数据结构
    if (instance->role == INTER_BOARD_ROLE_DOWN) {
        // 下板发给上板 (DOWN2UP)
        InterBoard_Down2UpMsg_t *msg = (InterBoard_Down2UpMsg_t *)(instance->tx_buffer);
        msg->control_mode = instance->app_data.tx_control_mode;
        msg->shoot_bool = instance->app_data.tx_shoot_bool;
        msg->up_target = InterBoard_FloatToHalf(instance->app_data.tx_target_up_yaw);
        msg->down_yaw_pos = InterBoard_FloatToHalf(instance->app_data.tx_current_down_yaw);
        msg->up_pitch_target = InterBoard_FloatToHalf(instance->app_data.tx_target_up_pitch);
    } 
    else {
        // 上板发给下板 (UP2DOWN)
        InterBoard_Up2DownMsg_t *msg = (InterBoard_Up2DownMsg_t *)(instance->tx_buffer);
        msg->findbool = instance->app_data.tx_find_bool;
        msg->up_yaw_pos = InterBoard_FloatToHalf(instance->app_data.tx_up_yaw_pos);
        msg->up_pitch_pos = InterBoard_FloatToHalf(instance->app_data.tx_up_pitch_pos);
    }

    // 调用抽象发送接口
    instance->tx_callback(instance, instance->tx_buffer, 8);
}

/**
 * @brief 核心接收：无论什么协议，均通过此门面将 payload 解析
 */
void InterBoard_ReceivePayload(InterBoard_Instance_t *instance, uint8_t* data, uint16_t len) {
    if(instance == NULL || data == NULL || len < 8) return;

    memcpy(instance->rx_buffer, data, 8);

    // 根据自身角色，解码收到的有效负载
    if (instance->role == INTER_BOARD_ROLE_DOWN) {
        // 我是下板，收到的必定是来自上板发的 Up2Down
        InterBoard_Up2DownMsg_t *msg = (InterBoard_Up2DownMsg_t *)(instance->rx_buffer);
        instance->app_data.rx_find_bool = msg->findbool;
        instance->app_data.rx_up_yaw_pos = InterBoard_HalfToFloat(msg->up_yaw_pos);
        instance->app_data.rx_up_pitch_pos = InterBoard_HalfToFloat(msg->up_pitch_pos);
    } 
    else {
        // 我是上板，收到的必定是来自下板发的 Down2Up
        InterBoard_Down2UpMsg_t *msg = (InterBoard_Down2UpMsg_t *)(instance->rx_buffer);
        instance->app_data.rx_control_mode = msg->control_mode;
        instance->app_data.rx_shoot_bool = msg->shoot_bool;
        instance->app_data.rx_target_up_yaw = InterBoard_HalfToFloat(msg->up_target);
        instance->app_data.rx_target_up_pitch = InterBoard_HalfToFloat(msg->up_pitch_target);
        instance->app_data.rx_current_down_yaw = InterBoard_HalfToFloat(msg->down_yaw_pos);
    }
}


/* ========================================================== */
/* CAN通信适配层 实现 */     
/* ========================================================== */

/**
 * @brief CAN协议发送胶水函数
 */
static void InterBoard_CAN_TxCallback(InterBoard_Instance_t* instance, uint8_t* data, uint16_t len) {
    CanInstance_s *can_inst = (CanInstance_s *)instance->transport_ctx;
    if (can_inst == NULL) return;

    memcpy(can_inst->tx_buff, data, len);
    if(!Can_Transmit(can_inst)) {
#ifdef DEBUG
        Log_Error("InterBoard CAN Tx failed\r\n");
#endif
    }
}

/**
 * @brief CAN协议作为底层的初始化
 */
InterBoard_Instance_t* InterBoard_Init_CAN(uint8_t can_number, uint8_t board_id) {
    // 实例化核心配置
    InterBoard_Config_t core_config = {0};
    core_config.board_id = board_id;
    core_config.tx_callback = InterBoard_CAN_TxCallback;
    
    // 初始化核心无关逻辑实例
    InterBoard_Instance_t* instance = InterBoard_Init(&core_config);
    if(instance == NULL) return NULL;

    // 分配和注册对应的CAN实例
    CanInitConfig_s can_config = {0};
    can_config.can_number = can_number;
    // 上板和下板使用交叉的ID收发。比如对于同一个ID（通常是1）：
    // 下板 tx_id = 0x0E+1=0x0F, rx_id = 0x0D+1=0x0E
    // 上板 tx_id = 0x0D+1=0x0E, rx_id = 0x0E+1=0x0F (反过来)
    if (instance->role == INTER_BOARD_ROLE_DOWN) {
        can_config.tx_id = 0x00E + board_id;
        can_config.rx_id = 0x00D + board_id;
    } else {
        can_config.tx_id = 0x00D + board_id;
        can_config.rx_id = 0x00E + board_id;
    }
    can_config.parent_ptr = instance; 
    can_config.can_module_callback = InterBoard_CAN_DecodeCallback; // 绑定本适配层的接收处理
    can_config.topic_name = "Inter_Board";

    instance->transport_ctx = Can_Register(&can_config);

    if (instance->transport_ctx == NULL) {
#ifdef DEBUG
        Log_Error("InterBoard can_wrapper failed\r\n");
#endif
    }
    
    return instance;
}

/**
 * @brief CAN解码回调胶水函数
 */
void InterBoard_CAN_DecodeCallback(CanInstance_s *can_instance) {
    if(can_instance == NULL || can_instance->parent_ptr == NULL) return;
    
    InterBoard_Instance_t *instance = (InterBoard_Instance_t *)can_instance->parent_ptr;

    // 从 CAN 接收缓存中提取 8Bytes 载荷，转交抽象接口解析
    InterBoard_ReceivePayload(instance, can_instance->rx_buff, 8);
}