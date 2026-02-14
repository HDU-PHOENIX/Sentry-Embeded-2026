/**
 * @file app_command_task.c
 * @author CGH
 * @brief 指令处理任务
 * @version V1.0.0
 * @note 有待进一步优化
 */
#include "app_command_task.h"

Dr16Instance_s *dr16_instance;
MiniPC_Instance *minipc_instance;
board_instance_t *board_instance;
Publisher *Command_publisher;
ShooterState_t Shooter_State;
ShooterState_t Shooter_State_last;

#define UP_DEBUG
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
uint8_t combined_state_global=0;//从下位机获取的combined值
uint16_t last_cnt=0,offline_time=0;
/**
 * @brief 根据遥控器的拨杆位置确定控制模式
 * @param dr16 指向遥控器实例的指针
 * @return uint8_t 返回当前的控制模式
 */
uint8_t Mode_Change(uint8_t combined_state_global){
  last_mode=mode;

   //从下位机直接获取combined后的值
  switch (combined_state_global)
  {
    case 0x11: // s1=1 (上), s2=1 (上)
      mode = last_mode==DISABLE_MODE?TRANS_MODE:PC_MODE;
			return mode;
      break;
    
    case 0x13: // s1=1 (上), s2=3 (中)
      
		mode = last_mode==DISABLE_MODE?TRANS_MODE:RC_MODE;
			
		return mode;
      break;

    case 0x12: // s1=1 (上), s2=2 (下)
      mode= last_mode==DISABLE_MODE?TRANS_MODE:UP_MODE;
      return mode;
      break;
    case 0x32: // s1=3 (中), s2=2 (中)
    if (last_mode == UP_MODE||last_mode == SHOOT_MODE)//之前忘记加SHOOT_MODE了，汗，导致他会自己切出到DISABLE_MODE
    {
      mode = SHOOT_MODE;
    }else{
    
      mode = DISABLE_MODE;
    }
		
      return mode;
      break;
    case 0x31:
    case 0x33:
			if(last_mode!=SCROP_MODE){
			mode = last_mode==RC_MODE?SCROP_MODE:DISABLE_MODE;
			}else{
				mode=SCROP_MODE;
			}
      return mode;
     

    default: // 其他所有情况 (包括 s1=2, s2在任意位置)
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
    #ifdef UP_DEBUG
    mode=TEST_MODE;
    #endif
    switch (mode)
    {
    case PC_MODE:
      if(Shooter_State_last==SHOOTER_STOP){
        Shooter_State=SHOOTER_TRANS;
      }else if(board_instance->received_shoot_bool==1){
      //Shooter_State=SHOOTER_AUTO;//暂时的逻辑
				Shooter_State=SHOOTER_TEST;//暂时的逻辑
      }else{
        Shooter_State=SHOOTER_READY;
      }
      break;
    case SHOOT_MODE:
    
			if(Shooter_State_last==SHOOTER_STOP){
					Shooter_State=SHOOTER_TRANS;
			}else if(board_instance->received_shoot_bool==1){
//      Shooter_State=SHOOTER_AUTO;//暂时的逻辑
						Shooter_State=SHOOTER_TEST;//暂时的逻辑
      }else{
        Shooter_State=SHOOTER_READY;
      }
      break;
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