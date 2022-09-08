# MD-6_firm
## commands
```cpp
      case 0x00:  //who am i 
         can.send(original_id, stm_CAN::ID_type::ext, stm_CAN::Frame_type::data, data, 0); 
         break; 
       case 0x01:  //hardware reset 
         HAL_NVIC_SystemReset(); 
         break; 
       case 0x02:  //set state 
         if (data[1] == 0x00){ 
           state = STATE_STOPPED; 
         } 
         else if (data[1] == 0x01){ 
           state = STATE_RUNNING; 
         } 
         break; 
       case 0x03:  //set recieve id 
         can.subscribe_message(data[1] | (data[2] << 8), stm_CAN::ID_type::std, stm_CAN::Frame_type::data, stm_CAN::FIFO::_1); 
         can.subscribe_message(data[3] | (data[4] << 8), stm_CAN::ID_type::std, stm_CAN::Frame_type::data, stm_CAN::FIFO::_1); 
         read_id[0] = data[1] | (data[2] << 8); 
         read_id[1] = data[3] | (data[4] << 8); 
         break; 
       case 0xff:  //API specify command 
         //0~7 :   set pwm 
         if(data[1] < 8 && state == STATE_RUNNING){ 
           motor_output[data[1]] = data[2] | (data[3] << 8); 
         } 
         break;
```
