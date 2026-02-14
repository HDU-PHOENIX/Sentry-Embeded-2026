/**
 * @file app_command_task.c
 * @author CGH
 * @brief 指令处理任务
 * @version V1.0.0
 */
#include "app_command_task.h"

Dr16Instance_s *dr16_instance;
MiniPC_Instance *minipc_instance;
MiniPC_Instance *minipc_blue_hp_instance;
MiniPC_Instance *minipc_sentry_status_instance;
Publisher *Command_publisher;
RefereeInstance_s* ref_instance;

//变量
uint8_t mode=0,last_mode=0;
uint8_t combined_state_global=0;
RefereeData_t RefreeData;

//配置

RefereeInitConfig_s referee_config = {
    .topic_name = "referee",
    .uart_handle = &huart10,
    .mode = UART_IDLE_MODE,
};


/**
 * @brief 根据遥控器的拨杆位置确定控制模式
 * @param dr16 指向遥控器实例的指针
 * @return uint8_t 返回当前的控制模式
 */
uint8_t Mode_Change(Dr16Instance_s *dr16){
  last_mode=mode;

   // 将s1的值左移4位，然后与s2的值进行'或'运算。
  // 这样s1占据高4位，s2占据低4位，形成一个唯一的8位状态值。
  // 例如: s1=1, s2=2  ->  (1 << 4) | 2  ->  0x10 | 0x02  ->  0x12
  uint8_t combined_state = (dr16->dr16_handle.s1 << 4) | dr16->dr16_handle.s2;
  combined_state_global=combined_state;
  switch (combined_state)
  {
    case 0x11: // s1=1 (上), s2=1 (上)
      mode = last_mode==DISABLE_MODE?TRANS_MODE:PC_MODE;
			return mode;
      break;
    
    case 0x13: // s1=1 (上), s2=3 (中)
      
      
      if(last_mode == PC_MODE || last_mode == DISABLE_MODE) {
          mode = TRANS_MODE;
      } else {
          mode = RC_MODE;
      }
			
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
	dr16_instance = Dr16_Register(&huart3); // 注册遥控器实例

  ref_instance = Referee_Register(&referee_config);
  if (ref_instance == NULL)
  {
      Log_Error("Referee Register Failed!");
  }

	Command_publisher=Create_Publisher("dr16_topic",sizeof(Dr16Instance_s));
	
  //如果代码不写死的话..
  RefreeData.color=Referee_Get_Color(ref_instance);
  /* Infinite loop */
  for(;;)
  {
		Publish_Message(Command_publisher, dr16_instance);
    mode=Mode_Change(dr16_instance);
    
		
    osDelay(2);
  }
  /* USER CODE END StartCommandTask */
}