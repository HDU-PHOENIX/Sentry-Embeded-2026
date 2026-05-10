/**
 * @file app_command_task.c
 * @author CGH
 * @brief ָ������� ���� �������� raw combined���������״̬��
 * @version V2.1.0
 */
#include "app_command_task.h"

Dr16Instance_s *dr16_instance;
MiniPC_Instance *minipc_instance;
board_instance_t *board_instance;
Publisher *Command_publisher;
ShooterState_t Shooter_State;
ShooterState_t Shooter_State_last;

board_config_t board_config = {
.board_id=1,
.can_config={
  .can_number =2,
  .topic_name = "Board_Comm"
},
.message_type = UP2DOWN_MESSAGE_TYPE
};

uint8_t mode=0,last_mode=0;
uint8_t combined_state_global=0;
uint16_t last_cnt=0,offline_time=0;

uint8_t Mode_Change(uint8_t combined){
    return DISABLE_MODE;
}

void StartCommandTask(void const * argument)
{
  /* USER CODE BEGIN StartCommandTask */
  Command_publisher=Create_Publisher("board_topic",sizeof(board_instance_t));
  board_instance = board_init(&board_config);
  
  /* Infinite loop */
  for(;;)
  {
    Publish_Message(Command_publisher, board_instance);
    
    // ֻ���� flag �ж�ʹ��/ʧ��
    if (board_instance->received_enable_flag == 1) {
        mode = UP_FOLLOW_MODE; // ʹ��ʱ��Ĭ�Ͻ��� UP_SHOOT_MODE ����Ħ����
    } else {
        mode = DISABLE_MODE;
    }

    Shooter_State_last=Shooter_State;
    //ͨ�Ŷ�ʧ�߼�
    if(board_instance->can_instance->cnt-last_cnt<1){
      offline_time++;
      if(offline_time>500){
        mode=DISABLE_MODE;
        Shooter_State=SHOOTER_STOP;
      }
    }else{
      offline_time=0;
    }
    offline_time=offline_time>2000?2000:offline_time;
    
    last_cnt=board_instance->can_instance->cnt;
    Shooter_State_last=Shooter_State;
    
    switch (mode)
    {
    case UP_FOLLOW_MODE:
      if(Shooter_State_last==SHOOTER_STOP){
        Shooter_State=SHOOTER_TRANS;
      }else {
        Shooter_State=SHOOTER_TEST;
      }
      break;

    case DISABLE_MODE:
    default:
      Shooter_State=SHOOTER_STOP;
      break;
    }
    osDelay(2);
  }
  /* USER CODE END StartCommandTask */
}
