/**
 * @file dev_minipc.c
 * @brief 上下位机通信
 * 
 * @details 该模块实现了上下位机之间的通信协议，包括数据包的定义和发送接收函数。
 * @note 实际上模块实现了不定长的数据包接收和解析，但是分包比较麻烦，这里又限制为了32个字节
 *
 * @author CGH
 * @editor HeWenXuan
 */


//牢大不允许我碎碎念（哭）

/**
 * 数据流过程：
 * 数据填入缓冲区→触发中断，转移到环形缓冲区→主动读取数据，转移到结构体→消息中心
 */
#include "dev_minipc.h"
#include <stdint.h>

#define USB_RX_BUFFER_SIZE 32//64个字节，2个完整包
#define USB_MAX_INSTANCE 8//允许的最多实例数量
uint16_t usb_cnt=0;//接收计数器
extern uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

//环形缓冲区,有数据到达会自动储存到这个缓冲区内

typedef struct {
    uint8_t buffer[USB_RX_BUFFER_SIZE];
    uint16_t head;
    uint16_t tail;
    uint16_t count;
} USB_RingBuffer_t;
extern USBD_HandleTypeDef hUsbDeviceFS; 
USB_RingBuffer_t usb_rx_ring_buffer = {0};
// static Publisher *USB_Topics[USB_MAX_INSTANCE] = {NULL};
static uint8_t USB_Instance_Count = 255;//当前USB实例数量
// 在文件顶部声明（修改变量类型）
static message_union* packet_pointer = NULL;  // 指针类型
MiniPC_Instance* minipc_instances[USB_MAX_INSTANCE] = {NULL};
extern USBD_CDC_ItfTypeDef USBD_Interface_fops_FS;


//做了个钩子函数，替换掉原来的USB接收回调函数，这样就能解耦了
void Minipc_HookInit(){
    //替换USB接收回调函数为我们自己的函数
 USBD_Interface_fops_FS.Receive =  USB_CDC_Hook_Callback;

}





/**
 * @brief 注册MiniPC实例
 * 
 * @param config MiniPC配置
 * @return MiniPC_Instance* 注册的实例指针
 */
MiniPC_Instance* Minipc_Register(MiniPC_Config* config) {
    // 检查实例数量限制
    uint8_t current_count = (USB_Instance_Count == 255) ? 0 : USB_Instance_Count;
    if (current_count >= USB_MAX_INSTANCE) {
#ifdef DEBUG
        Log_Error("Error: Maximum USB instances reached");
#endif
        return NULL;
    }
    
    // 分配内存
    MiniPC_Instance* instance = (MiniPC_Instance*)pvPortMalloc(sizeof(MiniPC_Instance));
    if(instance == NULL) {
#ifdef DEBUG
        Log_Error("Failed to allocate memory for MiniPC_Instance");
#endif
        return NULL;
    }

    //一个比较蠢的初始化逻辑。懒得修了
    if (USB_Instance_Count == 255) {
        USB_Instance_Count = 0;
    }
    
    instance->id = USB_Instance_Count;
    instance->callback = config->callback;
    instance->message_type = config->message_type;
    instance->activate_flag = 0;
    instance->Send_Message_Type = config->Send_message_type;
    
    // 初始化接收消息结构体（只用于接收）
    if (packet_pointer != NULL) {
        instance->message = *packet_pointer;
    } else {
        memset(&instance->message, 0, sizeof(message_union));
    }
    
    // 存储实例
    minipc_instances[USB_Instance_Count] = instance;
    
    // 递增计数器
    USB_Instance_Count++;
    
    // 注册USB接收回调函数
    USB_RegisterRxCallback(USB_Data_Received_Callback);

    return instance;
}

/**
 * @brief 设置指定消息类型的数据就绪标志
 * @param msg_type 消息类型
 */
static void USB_SetDataReadyFlags(uint8_t msg_type) {
    for (uint8_t i = 0; i < USB_Instance_Count+1; i++) {
        MiniPC_Instance* instance = minipc_instances[i];
        if (instance == NULL ) continue;

        // 检查这个实例是否关心这种消息类型
        
        if (instance->message_type == msg_type) {
            instance->activate_flag = 1; // 设置标志位
            instance->message = *packet_pointer; // 更新指针
            
        }else{
            instance->activate_flag = 0;
        }
        
    }
}

/**
 * @brief 从USB环形缓冲区读取数据
 * 
 * @param buffer 用于存储读取数据的缓冲区
 * @param max_len 缓冲区的最大长度
 * @return uint16_t 实际读取的字节数
 */
uint16_t Computer_Read_Data(uint8_t* buffer, uint16_t max_len) {
    uint16_t read_count = 0;
    
    if(buffer == NULL || max_len == 0) {
        return 0;
    }
    
    //TODO:可能需要临界区保护
    while(usb_rx_ring_buffer.count > 0 && read_count < max_len) {
        buffer[read_count] = usb_rx_ring_buffer.buffer[usb_rx_ring_buffer.tail];
        usb_rx_ring_buffer.tail = (usb_rx_ring_buffer.tail + 1) % USB_RX_BUFFER_SIZE;//自动绕回缓冲区开头
        usb_rx_ring_buffer.count--;
        read_count++;
    }
    
    return read_count;
}

// 检查缓冲区中是否有数据
static uint16_t Computer_Available_Data(void) {
    return usb_rx_ring_buffer.count;
}

/** 
 * @brief USB数据接收回调函数
 * 
 * @param buf 接收到的数据缓冲区
 * @param len 接收到的数据长度
 */
void USB_Data_Received_Callback(uint8_t* buf, uint32_t len) {
    if(buf == NULL || len == 0) {
        return;
    }
   
    usb_cnt++;
    usb_cnt=usb_cnt>65535?0:usb_cnt;
    for(uint32_t i = 0; i < len; i++) {
        if(usb_rx_ring_buffer.count < USB_RX_BUFFER_SIZE) {
            usb_rx_ring_buffer.buffer[usb_rx_ring_buffer.head] = buf[i];
            usb_rx_ring_buffer.head = (usb_rx_ring_buffer.head + 1) % USB_RX_BUFFER_SIZE;
            usb_rx_ring_buffer.count++;
        } else {
            // 缓冲区已满，可以考虑丢弃最老的数据或者报警
            // 这里选择丢弃新数据，也可以选择覆盖最老数据
            break;
        }
    }
    Data_Processing();
}

int8_t USB_CDC_Hook_Callback(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  
  // 如果有数据且回调函数已注册，则调用回调
  if(*Len > 0 ) {
    USB_Data_Received_Callback(Buf, *Len);
  }
  
  // 重新设置接收缓冲区，准备下次接收
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}
/**
 * @brief 处理接收到的USB数据
 * 
 * @note 该函数对外开放，以实现主动与被动的接受数据处理
 * @details 改进的数据包解析逻辑，避免数据丢失
 */
void Data_Processing(void) {
    static message_union packet;
    static uint8_t packet_state = 0;  // 0: 寻找包头, 1: 正在接收数据包
    static uint8_t packet_index = 0;  // 当前数据包接收索引
    uint8_t byte;
    
    while(Computer_Read_Data(&byte, 1) == 1) {
        switch(packet_state) {
            case 0: // 寻找包头
                if(byte == 's') {
                    // 清零数据包缓冲区
                    memset(packet.raw_data, 0, sizeof(packet.raw_data));
                    packet.raw_data[0] = 's';
                    packet_index = 1;
                    packet_state = 1;
                }
                break;
                
            case 1: // 接收数据包
                if(packet_index < 32) {  // 防止数组越界
                    packet.raw_data[packet_index] = byte;
                    packet_index++;

                    if(packet_index >= 32) {
                        // 接收完整个数据包，验证包尾
                        Log_Debug("Packet received: end byte = 0x%02X (should be 'e'=0x65)", packet.raw_data[31]);
                        packet_pointer=&packet;//把指针暴露出去喵
                        if(packet.raw_data[31] == 'e') {
                            // 处理接收到的有效数据包
                            Log_Passing("Valid packet received, type = 0x%02X", packet.raw_data[1]);
                            USB_SetDataReadyFlags(packet.raw_data[1]);
                        } else {
                            Log_Warning("Invalid packet: wrong end byte 0x%02X", packet.raw_data[31]);
                        }
                        // 重置状态，准备接收下一个数据包
                        packet_state = 0;
                        packet_index = 0;
                    }
                } else {
                    // 数据包长度异常，重置状态
                    Log_Warning("Packet length error, resetting state");
                    packet_state = 0;
                    packet_index = 0;
                }
                break;
        }
    }
}

/**
 * @brief 发送数据到上位机
 * 
 * @param yaw 偏航角
 * @param pitch 垂直角
 * @param color 敌方颜色
 * @param mode 模式
 * @param rune 符文标志
 * @note 如果不是哨兵，可以直接使用这个
 */
 //这个取名不太好主要是某个姓丁的飞机电控（笑）留下的遗产，我接手这个库完全是处于意外....就留着警示后人吧
void Computer_Tx(float yaw, float pitch, uint8_t color, uint8_t mode, uint8_t rune) {
    Computer_Tx_Message_t packet;//最好改个名字,发送也可以使用union

    // 填充固定字段
    packet.start = 's';
    packet.datatype = 0xB0;
    packet.high_gimbal_yaw = yaw;
    packet.low_gimbal_yaw =0.0;
    packet.pitch = pitch;
    packet.enemy_team_color = color;
    packet.mode = mode;
    packet.rune_flag = rune;
    packet.end = 'e';

    // 清空预留区域,填充为0
    memset(packet.reserved, 0, sizeof(packet.reserved));

    // 发送数据包
    CDC_Transmit_FS((uint8_t*)&packet,32);
}

/**
 * @brief 配置自瞄发送数据源 - 简化接口
 * @param instance 实例指针
 * @param high_yaw 云台yaw角指针
 * @param pitch 俯仰角指针  
 * @param enemy_color 敌方颜色指针
 * @param mode 模式指针
 * @param rune_flag 符标志指针
 * @param low_yaw 底盘yaw角指针
 */
void Minipc_ConfigAimTx(MiniPC_Instance* instance, float* high_yaw, float* pitch, uint8_t* enemy_color, uint8_t* mode, uint8_t* rune_flag, float* low_yaw) {
    if (instance == NULL || instance->Send_Message_Type != USB_MSG_AIM_TX) {
#ifdef DEBUG
        Log_Error("Error: Invalid instance for AIM_TX");
#endif
        return;
    }
    
    // 记录指针
    instance->data_source.aim_tx.high_gimbal_yaw = high_yaw;
    instance->data_source.aim_tx.pitch = pitch;
    instance->data_source.aim_tx.enemy_team_color = enemy_color;
    instance->data_source.aim_tx.mode = mode;
    instance->data_source.aim_tx.rune_flag = rune_flag;
    instance->data_source.aim_tx.low_gimbal_yaw = low_yaw;
    
    
}

/**
 * @brief 配置具有前馈的自瞄发送数据源
 * @param instance 实例指针
 * @param 
 * 
 */
void Minipc_ConfigExpAimTx(MiniPC_Instance* instance,uint8_t* mode,
    float* yaw,            
    float* yaw_vel,          
    float* pitch,               
    float* pitch_vel,           
    float* bullet_speed,     
    uint16_t* bullet_count) {
    if (instance == NULL || instance->Send_Message_Type != USB_MSG_EXP_AIM_TX) {
#ifdef DEBUG
        Log_Error("Error: Invalid instance for EXP_AIM_TX");
#endif
        return;
    }
    
    // 记录指针
    instance->data_source.exp_aim_tx.mode = mode;
    instance->data_source.exp_aim_tx.yaw = yaw;
    instance->data_source.exp_aim_tx.yaw_vel = yaw_vel;
    instance->data_source.exp_aim_tx.pitch = pitch;
    instance->data_source.exp_aim_tx.pitch_vel = pitch_vel;
    instance->data_source.exp_aim_tx.bullet_speed = bullet_speed;
    instance->data_source.exp_aim_tx.bullet_count = bullet_count;

    
    
    
}

#ifdef SENTRY_MODE
/**
 * @brief 配置友方位置1发送数据源 
 */
void Minipc_ConfigFriendPos1Tx(MiniPC_Instance* instance, float* inf3_x, float* inf3_y, float* inf4_x, float* inf4_y, float* inf5_x, float* inf5_y) {
    if (instance == NULL || instance->Send_Message_Type != USB_MSG_FRIEND1_TX) {
#ifdef DEBUG
        Log_Error("Error: Invalid instance for FRIEND1_TX");
#endif
        return;
    }
    
    instance->data_source.friend_pos_1.infantry_3_x = inf3_x;
    instance->data_source.friend_pos_1.infantry_3_y = inf3_y;
    instance->data_source.friend_pos_1.infantry_4_x = inf4_x;
    instance->data_source.friend_pos_1.infantry_4_y = inf4_y;
    instance->data_source.friend_pos_1.infantry_5_x = inf5_x;
    instance->data_source.friend_pos_1.infantry_5_y = inf5_y;
    
    
}

#endif

/**
 * @brief 配置红方血量发送数据源
 */
void Minipc_ConfigRedHPTx(MiniPC_Instance* instance, uint16_t* red1_hp, uint16_t* red2_hp, uint16_t* red3_hp, uint16_t* red4_hp, uint16_t* red5_hp, uint16_t* red7_hp) {
    if (instance == NULL || instance->Send_Message_Type != USB_MSG_RED_HP_TX) {
#ifdef DEBUG
        Log_Error("Error: Invalid instance for RED_HP_TX");
#endif
        return;
    }
    
    instance->data_source.red_hp.red_1_robot_HP = red1_hp;
    instance->data_source.red_hp.red_2_robot_HP = red2_hp;
    instance->data_source.red_hp.red_3_robot_HP = red3_hp;
    instance->data_source.red_hp.red_4_robot_HP = red4_hp;
    instance->data_source.red_hp.red_5_robot_HP = red5_hp;
    instance->data_source.red_hp.red_7_robot_HP = red7_hp;
}

/**
 * @brief 配置蓝方血量发送数据源
 */
void Minipc_ConfigBlueHPTx(MiniPC_Instance* instance, uint16_t* blue1_hp, uint16_t* blue2_hp, uint16_t* blue3_hp, uint16_t* blue4_hp, uint16_t* blue5_hp, uint16_t* blue7_hp) {
    if (instance == NULL || instance->Send_Message_Type != USB_MSG_BLUE_HP_TX) {
#ifdef DEBUG
        Log_Error("Error: Invalid instance for BLUE_HP_TX");
#endif
        return;
    }
    
    instance->data_source.blue_hp.blue_1_robot_HP = blue1_hp;
    instance->data_source.blue_hp.blue_2_robot_HP = blue2_hp;
    instance->data_source.blue_hp.blue_3_robot_HP = blue3_hp;
    instance->data_source.blue_hp.blue_4_robot_HP = blue4_hp;
    instance->data_source.blue_hp.blue_5_robot_HP = blue5_hp;
    instance->data_source.blue_hp.blue_7_robot_HP = blue7_hp;
}

/**
 * @brief 配置建筑血量发送数据源
 */
void Minipc_ConfigBuildingHPTx(MiniPC_Instance* instance, uint16_t* outpost_hp, uint16_t* base_hp) {
    if (instance == NULL || instance->Send_Message_Type != USB_MSG_BUILD_HP_TX) {
#ifdef DEBUG
        Log_Error("Error: Invalid instance for BUILD_HP_TX");
#endif
        return;
    }
    
    instance->data_source.building_hp.outpost_HP = outpost_hp;
    instance->data_source.building_hp.base_HP = base_hp;
}

/**
 * @brief 配置比赛信息发送数据源
 */
void Minipc_ConfigGameInfoTx(MiniPC_Instance* instance, uint8_t* enemy_color, uint8_t* game_progress, uint16_t* remain_time, uint16_t* gold_coin) {
    if (instance == NULL || instance->Send_Message_Type != USB_MSG_GAME_INFO_TX) {
#ifdef DEBUG
        Log_Error("Error: Invalid instance for GAME_INFO_TX");
#endif
        return;
    }
    
    instance->data_source.game_info.enemy_team_color = enemy_color;
    instance->data_source.game_info.game_progress = game_progress;
    instance->data_source.game_info.stage_remain_time = remain_time;
    instance->data_source.game_info.remaining_gold_coin = gold_coin;
}
    

/**
 * @brief 配置哨兵状态发送数据源
 */
void Minipc_ConfigSentryStatusTx(MiniPC_Instance* instance, uint16_t* current_hp, uint16_t* projectile_17mm) {
    if (instance == NULL || instance->Send_Message_Type != USB_MSG_SENTRY_STATUS_TX) {  // 假设哨兵状态用LAUNCH_TX或其他，需要确认
#ifdef DEBUG
        Log_Error("Error: Invalid instance for SENTRY_STATUS_TX");
#endif
        return;
    }
    
    instance->data_source.sentry_status.current_HP = current_hp;
    instance->data_source.sentry_status.projectile_17mm = projectile_17mm;
}

/**
 * @brief 更新单个实例的数据
 * @param instance 实例指针
 */
void Minipc_UpdateInstanceData(MiniPC_Instance* instance) {
     // 检查USB设备是否已由主机配置。如果未配置，则不发送数据。
    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        return; // USB未连接或未准备好，直接返回
    }

    if (instance == NULL) {
        return;
    }
    
    // 动态分配发送缓冲区
    send_union* send_buffer = (send_union*)pvPortMalloc(sizeof(send_union));
    if (send_buffer == NULL) {
#ifdef DEBUG
        Log_Error("Failed to allocate send buffer for instance %d", instance->id);
#endif
        return;
    }
    
    // 初始化发送缓冲区
    memset(send_buffer, 0, sizeof(send_union));
    
    // 配置包头包尾
    USB_INIT_MESSAGE_PACKET(send_buffer, instance->Send_Message_Type);
    
    switch (instance->Send_Message_Type) {
        case USB_MSG_AIM_TX: {
            Computer_Tx_Message_t* msg = &send_buffer->self_aim_pack;
            USB_AimTx_DataSource_t* src = &instance->data_source.aim_tx;
            
            // 填充数据
            if (src->high_gimbal_yaw != NULL) msg->high_gimbal_yaw = *(src->high_gimbal_yaw);
            if (src->pitch != NULL) msg->pitch = *(src->pitch);
            if (src->enemy_team_color != NULL) msg->enemy_team_color = *(src->enemy_team_color);
            if (src->mode != NULL) msg->mode = *(src->mode);
            if (src->rune_flag != NULL) msg->rune_flag = *(src->rune_flag);
            if (src->low_gimbal_yaw != NULL) msg->low_gimbal_yaw = *(src->low_gimbal_yaw);
            
            break;
        }
        case USB_MSG_EXP_AIM_TX: {
            exp_tx_aim_package* msg = &send_buffer->exp_aim_pack;
            USB_ExpAimTx_DataSource_t* src = &instance->data_source.exp_aim_tx;
            
            // 填充数据
            if (src->mode != NULL) msg->mode = *(src->mode);
            if (src->yaw != NULL) msg->yaw = *(src->yaw);
            if (src->yaw_vel != NULL) msg->yaw_vel = *(src->yaw_vel);
            if (src->pitch != NULL) msg->pitch = *(src->pitch);
            if (src->pitch_vel != NULL) msg->pitch_vel = *(src->pitch_vel);
            if (src->bullet_speed != NULL) msg->bullet_speed = *(src->bullet_speed);
            if (src->bullet_count != NULL) msg->bullet_count = *(src->bullet_count);
            
            break;
        }
        #ifdef SENTRY_MODE
        case USB_MSG_FRIEND1_TX: {
            friendly_position_1_package* msg = &send_buffer->friend_pos_1;
            USB_FriendPos1_DataSource_t* src = &instance->data_source.friend_pos_1;
            
            if (src->infantry_3_x != NULL) msg->infantry_3_x = *(src->infantry_3_x);
            if (src->infantry_3_y != NULL) msg->infantry_3_y = *(src->infantry_3_y);
            if (src->infantry_4_x != NULL) msg->infantry_4_x = *(src->infantry_4_x);
            if (src->infantry_4_y != NULL) msg->infantry_4_y = *(src->infantry_4_y);
            if (src->infantry_5_x != NULL) msg->infantry_5_x = *(src->infantry_5_x);
            if (src->infantry_5_y != NULL) msg->infantry_5_y = *(src->infantry_5_y);
            break;
        }
        case USB_MSG_RED_HP_TX: {
            red_hp_package* msg = &send_buffer->red_hp;
            USB_RedHP_DataSource_t* src = &instance->data_source.red_hp;
            
            if (src->red_1_robot_HP != NULL) msg->red_1_robot_HP = *(src->red_1_robot_HP);
            if (src->red_2_robot_HP != NULL) msg->red_2_robot_HP = *(src->red_2_robot_HP);
            if (src->red_3_robot_HP != NULL) msg->red_3_robot_HP = *(src->red_3_robot_HP);
            if (src->red_4_robot_HP != NULL) msg->red_4_robot_HP = *(src->red_4_robot_HP);
            if (src->red_5_robot_HP != NULL) msg->red_5_robot_HP = *(src->red_5_robot_HP);
            if (src->red_7_robot_HP != NULL) msg->red_7_robot_HP = *(src->red_7_robot_HP);
            break;
        }
        case USB_MSG_BLUE_HP_TX: {
            blue_hp_package* msg = &send_buffer->blue_hp;
            USB_BlueHP_DataSource_t* src = &instance->data_source.blue_hp;
            
            if (src->blue_1_robot_HP != NULL) msg->blue_1_robot_HP = *(src->blue_1_robot_HP);
            if (src->blue_2_robot_HP != NULL) msg->blue_2_robot_HP = *(src->blue_2_robot_HP);
            if (src->blue_3_robot_HP != NULL) msg->blue_3_robot_HP = *(src->blue_3_robot_HP);
            if (src->blue_4_robot_HP != NULL) msg->blue_4_robot_HP = *(src->blue_4_robot_HP);
            if (src->blue_5_robot_HP != NULL) msg->blue_5_robot_HP = *(src->blue_5_robot_HP);
            if (src->blue_7_robot_HP != NULL) msg->blue_7_robot_HP = *(src->blue_7_robot_HP);
            break;
        }
        case USB_MSG_BUILD_HP_TX: {
            building_hp_package* msg = &send_buffer->building_hp;
            USB_BuildingHP_DataSource_t* src = &instance->data_source.building_hp;
            
            if (src->outpost_HP != NULL) msg->outpost_HP = *(src->outpost_HP);
            if (src->base_HP != NULL) msg->base_HP = *(src->base_HP);
            break;
        }
        case USB_MSG_GAME_INFO_TX: {
            game_info_package* msg = &send_buffer->game_info;
            USB_GameInfo_DataSource_t* src = &instance->data_source.game_info;
            
            if (src->enemy_team_color != NULL) msg->enemy_team_color = *(src->enemy_team_color);
            if (src->game_progress != NULL) msg->game_progress = *(src->game_progress);
            if (src->stage_remain_time != NULL) msg->stage_remain_time = *(src->stage_remain_time);
            if (src->remaining_gold_coin != NULL) msg->remaining_gold_coin = *(src->remaining_gold_coin);
            break;
        }
        case USB_MSG_SENTRY_STATUS_TX: {
             Sentry_Status_Tx_Package_t* msg = &send_buffer->sentry_status;
            USB_DataSource_Sentry_Status_t* src = &instance->data_source.sentry_status;
            
            if (src->current_HP != NULL) msg->current_HP = *(src->current_HP);
            if (src->projectile_17mm != NULL) msg->projectile_17mm = *(src->projectile_17mm);
            break;
        }
        #endif
        case DIY_MODE:{
            //这里可以加入自定义的回调函数处理
            instance->callback(send_buffer);
        }
        default:
            vPortFree(send_buffer);
            return;
    }
    
    // 发送数据
    CDC_Transmit_FS((uint8_t*)send_buffer, 32);
    
    // 立即释放内存
    vPortFree(send_buffer);
}

/**
 * @brief 更新所有已配置数据源的发送实例 
 */
void Minipc_UpdateAllInstances(void) {
    
    
    uint8_t actual_count = (USB_Instance_Count == 0) ? 1 : USB_Instance_Count;
    
    for (uint8_t i = 0; i < actual_count; i++) {
        MiniPC_Instance* instance = minipc_instances[i];
        
        
        if (instance != NULL) {
            if(instance->Send_Message_Type==NULL){
                
                continue;
            }

            Minipc_UpdateInstanceData(instance);
            osDelay(1);
        }
    }
}





