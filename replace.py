import re

with open('Task/app_command_task.c', 'r', encoding='utf-8', errors='ignore') as f:
    content = f.read()

replacement = \"\"\"
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
    
    // 只根据 flag 判断使能/失能
    if (board_instance->received_enable_flag == 1) {
        mode = UP_SHOOT_MODE; // 使能时，进入 UP_SHOOT_MODE (云台使能，且摩擦轮持续旋转)
    } else {
        mode = DISABLE_MODE;
    }

    Shooter_State_last=Shooter_State;
    //通信丢失逻辑
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
    case UP_SHOOT_MODE:
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
\"\"\"

# Regex replace from "uint8_t Mode_Change" down to the end of the file.
pattern = re.compile(r'uint8_t Mode_Change\(uint8_t combined\).*', re.DOTALL)
if pattern.search(content):
    new_content = pattern.sub(replacement.strip(), content)
    with open('Task/app_command_task.c', 'w', encoding='utf-8') as f:
        f.write(new_content)
    print("Success")
else:
    print("Failed to find block")
