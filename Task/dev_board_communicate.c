/**
 * @file dev_board_communicate.c
 * @author CGH
 * @brief 板间通信
 * 
 */
#include "dev_board_communicate.h"
#include <string.h>

// 函数声明
static uint16_t float2half(float rawdata);
static float half2float(uint16_t halfdata);

board_instance_t* board_init(board_config_t *config)
{
    if(config == NULL)
    {
        #ifdef DEBUG
        Log_Error("Board config is NULL\r\n");
        #endif
        return NULL;
    }
    // 动态分配内存
    board_instance_t* instance = (board_instance_t*)pvPortMalloc(sizeof(board_instance_t));
    if (instance == NULL)
    {
        #ifdef DEBUG
        Log_Error("Failed to allocate memory for board instance\r\n");
        #endif
        return NULL;
    }
    memset(instance, 0, sizeof(board_instance_t));
  
    config->can_config.tx_id = BOARD_TX_ID_BASE + config->board_id;
    config->can_config.rx_id = BOARD_RX_ID_BASE + config->board_id;
    config->can_config.parent_ptr = instance; // 父指针指向实例本身
    config->can_config.can_module_callback = Board_Message_Decode;
    config->can_config.topic_name = "Board_Comm";
    instance->board_id = config->board_id;
    instance->can_instance = Can_Register(&config->can_config);
    
    instance->message_type = config->message_type;
    return instance; // 返回分配好的实例的指针
}

void board_send_message(board_instance_t *instance, float data1, float data2, uint8_t flag1, uint8_t flag2)
{
    if(instance == NULL || instance->can_instance == NULL)
    {
        #ifdef DEBUG
        Log_Error("Board instance or CAN instance is NULL\r\n");
        #endif
        return;
    }

    // 根据消息类型，使用宏填充8字节的数据缓冲区
    switch (instance->message_type)
    {
        case 0: // 发送 down2up_message_t
        {
            down2up_message_t *msg = ACCESS_AS_STRUCT(instance, down2up_message_t);
            msg->control_mode = flag1;
            msg->shoot_bool = flag2;
            msg->up_target = float2half(data1);
            msg->down_yaw_pos = float2half(data2);
            break;
        }
        case 1: // 发送 up2down_message_t
        {
            up2down_message_t *msg = ACCESS_AS_STRUCT(instance, up2down_message_t);
            msg->findbool = flag1;
            msg->ready_bool = flag2;
            msg->up_yaw_pos = float2half(data1);
            msg->up_pitch_pos = float2half(data2);
            break;
        }
        default:
            #ifdef DEBUG
            Log_Error("Unknown message type for board ID %d\r\n", instance->board_id);
            #endif
            return;
    }

    // 将填充好的8字节缓冲区数据拷贝到CAN发送缓冲区并发送
    memcpy(instance->can_instance->tx_buff, instance->data_buffer, 8);
    if(!Can_Transmit(instance->can_instance))
    {
        #ifdef DEBUG
        Log_Error("Board ID %d message send failed\r\n", instance->board_id);
        #endif
    }
}

void Board_Message_Decode(CanInstance_s *can_instance)
{
    if(can_instance == NULL || can_instance->parent_ptr == NULL)
    {
        return;
    }
    board_instance_t *board_instance = (board_instance_t *)can_instance->parent_ptr;

    // 将CAN接收缓冲区的8字节数据拷贝到实例的数据缓冲区
    memcpy(board_instance->data_buffer, can_instance->rx_buff, 8);

    // 根据消息类型，解码数据并存放到解析后的成员中
    switch (board_instance->message_type)
    {
        case 0: // 接收到 up2down_message_t (下板接收上板发来的数据)
        {
            
            up2down_message_t *msg = ACCESS_AS_STRUCT(board_instance, up2down_message_t);
            board_instance->received_find_bool = msg->findbool;
            board_instance->received_ready_bool = msg->ready_bool;
            board_instance->received_up_yaw_pos = half2float(msg->up_yaw_pos);
            board_instance->received_up_pitch_pos = half2float(msg->up_pitch_pos);
            break;
        }
        case 1: // 接收到 up2down_message_t
        {
            down2up_message_t *msg = ACCESS_AS_STRUCT(board_instance, down2up_message_t);
            board_instance->received_control_mode = msg->control_mode;
            board_instance->received_shoot_bool = msg->shoot_bool;
            board_instance->received_target_up_yaw = half2float(msg->up_target);
            board_instance->received_target_up_pitch = half2float(msg->up_pitch_target);
            board_instance->received_current_down_yaw = half2float(msg->down_yaw_pos);
            break;
        }
        default:
            #ifdef DEBUG
            Log_Error("Unknown message type for board ID %d\r\n", board_instance->board_id);
            #endif
            return;
    }

}

//0000 0000 0000 0001，将第一位移到最后
//0000 0000 1111 1111,取最后8位，也就是32位移位23位后的前8位被取
//0000 0111 1111 1111 1111 1111 1111,取了23位，也就取到了尾数位

static uint16_t float2half(float rawdata)
{
    uint16_t result;
    FloatUnion fu;
    fu.f = rawdata; 
    uint32_t f32_bits = fu.u;

    uint32_t sign_f32 = (f32_bits >> 31) & 0x01;
    uint32_t exp_f32 = (f32_bits >> 23) & 0xFF;
    uint32_t mant_f32 = f32_bits & 0x7FFFFF;

    // 处理零值
    if(exp_f32 == 0){
        result = (sign_f32 << 15);
        return result;
    }
    
    // 处理无穷大和NaN
    if(exp_f32 == 255){
        result = (sign_f32 << 15) | 0x7C00 | (mant_f32 >> 13);
        return result;
    }
    
    // 下溢，返回0
    if(exp_f32 < 113){
        result = (sign_f32 << 15);
        return result;
    }
    
    // 上溢，返回无穷大
    if(exp_f32 > 142){
        result = (sign_f32 << 15) | 0x7C00;
        return result;
    }
    
    // 正常转换
    result = (uint16_t)(sign_f32 << 15) | ((exp_f32 - 112) << 10) | (mant_f32 >> 13);
    return result;
}
//float的结构：1位符号位，8位指数位，23位尾数位
//half的结构：1位符号位，5位指数位，10位尾数位
//0000 0011 1111 1111（也就是0x3FF）作为掩码取低10位
//0000 0001 1111 0000(也就是0x1F0)作为掩码取中间5位
static float half2float(uint16_t halfdata)
{
    float result;
    uint32_t sign = (halfdata >> 15) & 0x1;
    uint32_t exp = (halfdata >> 10) & 0x1F;
    uint32_t mant = halfdata & 0x3FF;
    
    uint32_t floatdata;
    
    // 处理零值
    if(exp == 0 && mant == 0){
        floatdata = (sign << 31);
    }
    // 处理无穷大和NaN
    else if(exp == 0x1F){
        floatdata = (sign << 31) | (0xFF << 23) | (mant << 13);
    }
    // 正常值
    else{
        floatdata = (sign << 31) | ((exp - 15 + 127) << 23) | (mant << 13);
    }
    
    memcpy(&result, &floatdata, sizeof(float));
    return result;
}