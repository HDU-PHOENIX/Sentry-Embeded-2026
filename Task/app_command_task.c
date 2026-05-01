/**
 * @file app_command_task.c
 * @author CGH
 * @brief 指令处理任务 —— 独立解析 raw combined，驱动射击状态机
 * @version V2.1.0
 * @note 上板独立运行 Mode_Change，避免下板 TRANS_MODE 丢包导致切换不平滑
 */
#include "app_command_task.h"

Dr16Instance_s *dr16_instance;
MiniPC_Instance *minipc_instance;
board_instance_t *board_instance;
Publisher *Command_publisher;
ShooterState_t Shooter_State;
ShooterState_t Shooter_State_last;

//#define UP_DEBUG
//配置
board_config_t board_config = {
.board_id=1,
.can_config={
  .can_number =2,//记得改回来
  .topic_name = "Board_Comm"
},
.message_type = UP2DOWN_MESSAGE_TYPE



};


//变量
uint8_t mode=0,last_mode=0;
uint8_t combined_state_global=0;//下板发来的 raw combined (s1<<4 | s2)
uint16_t last_cnt=0,offline_time=0;

/**
 * @brief 根据 raw combined 独立解析上板控制模式
 * @param combined 下板发来的 raw combined = (s1<<4) | s2
 * @return uint8_t 当前 SentryMode_t
 * @note 逻辑与下板 Clean_Down 完全一致，确保 TRANS_MODE 在上板本地平滑过渡
 *       上板只关心云台相关模式，底盘模式统一 DISABLE_MODE
 */
uint8_t Mode_Change(uint8_t combined){
    static uint8_t general_mode = DISABLE_MODE;
    last_mode = mode;

    switch (combined) {
        case 0x12: // s1=1(上), s2=2(下) → 导航模式
            mode = (last_mode == DISABLE_MODE) ? TRANS_MODE : PC_MODE;
            general_mode = PC_MODE;
            break;
        case 0x13: // s1=1(上), s2=3(中) → 手动模式
            mode = (last_mode == DISABLE_MODE) ? TRANS_MODE : HANDLE_MODE;
            general_mode = HANDLE_MODE;
            break;
        case 0x11: // s1=1(上), s2=1(上) → 失能
            mode = DISABLE_MODE;
            general_mode = DISABLE_MODE;
            break;

        /* ---- s1=2(下)：发射域 ---- */
        case 0x21: // s1=2(下), s2=1(上) → 自瞄
            if(general_mode == HANDLE_MODE){
                mode = (last_mode == DISABLE_MODE) ? TRANS_MODE : UP_SHOOT_AUTO_MODE;
            }else if(general_mode == PC_MODE){
                mode = (last_mode == DISABLE_MODE) ? TRANS_MODE : UP_SHOOT_PC_MODE;
            }
            break;
        case 0x23: // s1=2(下), s2=3(中) → 连发
            if(general_mode == HANDLE_MODE){
                if(last_mode == DISABLE_MODE) {
                    mode = TRANS_MODE;
                }else if(last_mode == UP_FOLLOW_MODE || last_mode == UP_SHOOT_MODE){
                    mode = UP_SHOOT_MODE;
                }else{
                    mode = DISABLE_MODE;
                }
            }else if(general_mode == PC_MODE){
                mode = (last_mode == DISABLE_MODE) ? TRANS_MODE : UP_SHOOT_PC_MODE;
            }
            break;
        case 0x22: // s1=2(下), s2=2(下) → 单发/比赛
            if(general_mode == PC_MODE){
                mode = (last_mode == DISABLE_MODE) ? TRANS_MODE : CHASSIS_COMPETATION_MODE;
            }else{
                //手动域：单发模式，当前硬件暂不支持，一律失能
                mode = DISABLE_MODE;
            }
            break;

        /* ---- s1=3(中)：底盘域，上板不关心云台，统一失能 ---- */
        case 0x31: // s1=3(中), s2=1(上)
            if(general_mode == HANDLE_MODE){
                mode = (last_mode == DISABLE_MODE) ? TRANS_MODE : UP_FOLLOW_MODE;
            }else if(general_mode == PC_MODE){
                mode = DISABLE_MODE; // CHASSIS_AUTOAIM_PC_MODE，上板不关心
            }
            break;
        case 0x33: // s1=3(中), s2=3(中)
            if(general_mode == HANDLE_MODE){
                mode = (last_mode == DISABLE_MODE) ? TRANS_MODE : CHASSIS_FOLLOW_MODE;
            }else if(general_mode == PC_MODE){
                mode = DISABLE_MODE; // CHASSIS_FOLLOW_PC_MODE，上板不关心
            }
            break;
        case 0x32: // s1=3(中), s2=2(下)
            if(general_mode == HANDLE_MODE){
                mode = (last_mode == DISABLE_MODE) ? TRANS_MODE : CHASSIS_GYRO_MODE;
            }else if(general_mode == PC_MODE){
                mode = DISABLE_MODE; // CHASSIS_COMPETATION_MODE，上板不关心
            }
            break;

        default:
            mode = DISABLE_MODE;
            break;
    }
    return mode;
}

void StartCommandTask(void const * argument)
{
  /* USER CODE BEGIN StartCommandTask */
	
	Command_publisher=Create_Publisher("board_topic",sizeof(board_instance_t));
  board_instance = board_init(&board_config);
  
  /* Infinite loop */
  //写状态机的时候注意，不是切换了模式就可以了，要考虑程序会继续运行完这个分支！
  for(;;)
  {
		Publish_Message(Command_publisher, board_instance);
    combined_state_global=board_instance->received_control_mode;
    mode=Mode_Change(combined_state_global);
    Shooter_State_last=Shooter_State;
    //通信丢失逻辑，其实就是个替代看门狗的玩意。
    if(board_instance->can_instance->cnt-last_cnt<1){
      //下位机不再发包，通信丢失
      offline_time++;
      if(offline_time>500){
      mode=DISABLE_MODE;
      Shooter_State=SHOOTER_STOP;
      }
    }else{
      offline_time=0;
    }
    offline_time=offline_time>2000?2000:offline_time;//避免溢出回绕
    
    last_cnt=board_instance->can_instance->cnt;
    Shooter_State_last=Shooter_State;
    switch (mode)
    {
    /* ---- 需要射击的模式：摩擦轮使能 ---- */
    case PC_MODE:
    case UP_SHOOT_MODE:
    case UP_SHOOT_AUTO_MODE:
    case UP_SHOOT_PC_MODE:
      if(Shooter_State_last==SHOOTER_STOP){
        Shooter_State=SHOOTER_TRANS;
      }else {
        Shooter_State=SHOOTER_TEST;
      }
      break;
    /* ---- 不需要射击的模式：摩擦轮失能（云台仍使能） ---- */
    case HANDLE_MODE:
    case UP_FOLLOW_MODE:
    case UP_SCORPE_MODE:
    case CHASSIS_LOCK_MODE:
    case CHASSIS_GYRO_MODE:
    case CHASSIS_FOLLOW_MODE:
    case CHASSIS_FOLLOW_PC_MODE:
    case CHASSIS_AUTOAIM_PC_MODE:
    case CHASSIS_COMPETATION_MODE:
      Shooter_State=SHOOTER_STOP;
      break;
    /* ---- 全车失能 ---- */
    case DISABLE_MODE:
      Shooter_State=SHOOTER_STOP;
      break;
    
    default:
      Shooter_State=SHOOTER_STOP;
      break;
    }
    osDelay(2);
  }
  /* USER CODE END StartCommandTask */
}