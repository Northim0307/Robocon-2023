//Robot Direction
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFTWARD 3
#define RIGHTWARD 4
#define ROTATE_CW 5 
#define ROTATE_C_CW 6

#define MOVING 1
#define STOPPING 0

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
#define HALT 2

#define STANDARD_CAN_FRAME 0

//MPU9250
#define MPU9250_IMU_ADDRESS 0x68
#define INTERVAL_MS_PRINT 1000

struct can_frame {
    unsigned long   can_id;       // CAN_ID 
    byte            can_dlc;      // frame data length in byte 
    byte            data[8]={0};  // CAN_DATA 
}
can_output,         //CAN struct to store id,dlc and data for output from MCU to Speed controller
can_input,          //CAN struct to store id,dlc and data for input  from Speed controller to MCU
can_motor_msg[4];   //CAN struct to store id,dlc and data for M1 and M2

int previous_direction,i;

MCP_CAN CAN0(5);     // Set CS pin to pin 5

MPU9250 mpu;


void Robot_Direction(int direction, int motor_direction[4])
{
	if(direction == STOP)
	{
		motor_direction[0] = HALT;   
		motor_direction[1] = HALT;
		motor_direction[2] = HALT;
		motor_direction[3] = HALT;
	}
		
	if(direction == FORWARD)
	{
		motor_direction[0] = COUNTER_CLOCKWISE;   //counter_clockwise but current value is positive for the DJI to spin counter clockwise
		motor_direction[1] = CLOCKWISE;
		motor_direction[2] = CLOCKWISE;
		motor_direction[3] = COUNTER_CLOCKWISE;
	}
	
	if(direction == BACKWARD)
	{
		motor_direction[0] = CLOCKWISE;
		motor_direction[1] = COUNTER_CLOCKWISE;
		motor_direction[2] = COUNTER_CLOCKWISE;
		motor_direction[3] = CLOCKWISE;
	}
	
	if(direction == LEFTWARD)
	{
		motor_direction[0] = CLOCKWISE;
		motor_direction[1] = CLOCKWISE;
		motor_direction[2] = COUNTER_CLOCKWISE;
		motor_direction[3] = COUNTER_CLOCKWISE;
	}
	
	if(direction == RIGHTWARD)
	{
		motor_direction[0] = COUNTER_CLOCKWISE;
		motor_direction[1] = COUNTER_CLOCKWISE;
		motor_direction[2] = CLOCKWISE;
		motor_direction[3] = CLOCKWISE;
	}
	
	if(direction == ROTATE_CW)
	{
		motor_direction[0] = COUNTER_CLOCKWISE;
		motor_direction[1] = COUNTER_CLOCKWISE;
		motor_direction[2] = COUNTER_CLOCKWISE;
		motor_direction[3] = COUNTER_CLOCKWISE;
	}
	
	if(direction == ROTATE_C_CW)
	{
		motor_direction[0] = CLOCKWISE;
		motor_direction[1] = CLOCKWISE;
		motor_direction[2] = CLOCKWISE;
		motor_direction[3] = CLOCKWISE;
	}
	
	return ;
	
}

void CAN_receive_msg ( uint16_t feedback_motor[4] )
{
	int retry = 0; 
	
	//flags to indicate whether the feedback for both motor are received
	bool flag_motor1 = false ;
	bool flag_motor2 = false ;
	bool flag_motor3 = false ;
	bool flag_motor4 = false ;
	
	while(retry<20)
	{
		CAN0.readMsgBuf(&can_input.can_id, &can_input.can_dlc, can_input.data);      // Read data: len = data length, buf = data byte(s)
		switch(can_input.can_id)
		{
			case 0x201:
			{
				can_motor_msg[0]=can_input;
				feedback_motor[0] = (unsigned int) can_input.data[3]|(can_input.data[2]<<8);
				flag_motor1 = 1;
				break;
			}
			case 0x202:
			{
				can_motor_msg[1]=can_input;
				feedback_motor[1] = (unsigned int)  can_input.data[3]|(can_input.data[2]<<8);
				flag_motor2 = 1;
				break;
			}
			case 0x203:
			{
				can_motor_msg[2]=can_input;
				feedback_motor[2] = (unsigned int)  can_input.data[3]|(can_input.data[2]<<8);
				flag_motor3 = 1;
				break;
			}
			case 0x204:
			{
				can_motor_msg[3]=can_input;
				feedback_motor[3] = (unsigned int)  can_input.data[3]|(can_input.data[2]<<8);
				flag_motor4 = 1;
				break;
			}
		}
    if(flag_motor1&&flag_motor2&&flag_motor3&&flag_motor4)
		break;
    else
      retry++;
  }
  
  for (i = 0 ; i < 4 ; i++)
  {
    if(feedback_motor[i]>32767)
    feedback_motor[i]=(~feedback_motor[i])+1;
  }
  return;
}

void CAN_send_msg(int motor_direction[4], double output_motor[4])
{
	double CAN_Output_M[4];
	
	for(i = 0 ; i < 4 ; i++)
    {
      if (motor_direction[i]== COUNTER_CLOCKWISE)              
      {
        CAN_Output_M[i] = output_motor[i] ;
        //Output_M[i] = Output_M[i] ;       //positive is counterclockwise
      }
      else
      {
        CAN_Output_M[i] = (-output_motor[i]);
        //Output_M[i] = (-Output_M[i]);
      }
    }
	
	can_output.data[0]= (unsigned int) (CAN_Output_M[0]) >> 8;
	can_output.data[1]= (unsigned int) (CAN_Output_M[0]) ;
	can_output.data[2]= (unsigned int) (CAN_Output_M[1]) >> 8;
	can_output.data[3]= (unsigned int) (CAN_Output_M[1]) ;
	can_output.data[4]= (unsigned int) (CAN_Output_M[2]) >> 8;
	can_output.data[5]= (unsigned int) (CAN_Output_M[2]) ;
	can_output.data[6]= (unsigned int) (CAN_Output_M[3]) >> 8;
	can_output.data[7]= (unsigned int) (CAN_Output_M[3]) ;
	
	// send data:  ID , Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
	byte sndStat = CAN0.sendMsgBuf(can_output.can_id, STANDARD_CAN_FRAME, can_output.can_dlc, can_output.data);
	if(sndStat == CAN_OK)
	{
		//    Serial.println("Message Sent Successfully!");
	} 
	else 
	{
		Serial.println("Error Sending Message...");
	}
	return;
}

// void PID_Automatic(PID pid_m1,PID pid_m2, PID pid_m3, PID pid_m4, int motor_num)
// {
	// bool PID_completed = false;
	// while(PID_completed==false)
	// {
		// switch(motor_num)
        // {
          // case 0 :
          // pid_m1.SetMode(AUTOMATIC);
          // PID_completed = pid_m1.Compute();
          // break; 
          
          // case 1 :
          // pid_m2.SetMode(AUTOMATIC);
          // PID_completed = pid_m2.Compute();
          // break; 
           
          // case 2 :
          // pid_m3.SetMode(AUTOMATIC);
          // PID_completed = pid_m3.Compute();
          // break;  
          
          // case 3 :
          // pid_m4.SetMode(AUTOMATIC);
          // PID_completed = pid_m4.Compute();
          // break;           
        // }
	// }
	// return;
// }

void PrintFeedback (uint16_t feedback_msg[4])
{
	Serial.print(feedback_msg[0]);
	Serial.print(" ,");
	Serial.print(feedback_msg[1]);
	Serial.print(" ,");
	Serial.print(feedback_msg[2]);
	Serial.print(" ,");
	Serial.println(feedback_msg[3]);
	
}
	
	
double calc_optimum_current ( double current, int sample_size, int direction, int motor_num )
{
	static int count[5][4] = {0};
	static double optimum_current[5][4] = {0};
	
	if (count[direction][motor_num] < sample_size)
	{
		Serial.print(" Count now for Motor : ");
		Serial.print(motor_num+1);
		Serial.print(" in direction of :  ");
		Serial.print(direction);
		Serial.print(" is :");
		Serial.println(count[direction][motor_num]);
		
		optimum_current[direction][motor_num] += current;
		count[direction][motor_num]++;
		if(count[direction][motor_num] == sample_size)
		{
			optimum_current[direction][motor_num] /= sample_size;
		}
	}
	
	if(count[direction][motor_num] == sample_size)
	{
		Serial.print(" Optimum Current of Motor : ");
		Serial.print(motor_num+1);
		Serial.print(" in direction of :  ");
		Serial.print(direction);
		Serial.print(" is :");
		Serial.println(optimum_current[direction][motor_num]);
		
		return optimum_current[direction][motor_num];
	}
	else
	{
		return 0;
	}
}
	
void getInitialYaw(float *initial_yaw)
{
	float yaw_difference,yaw_now,yaw_prev;
	unsigned long currentMillis = 0, lastPrintMillis = 0;
	i = 0;
	while ( initial_yaw == 0)
  {
    currentMillis = millis();
    if (mpu.update()&& currentMillis - lastPrintMillis > INTERVAL_MS_PRINT ) {
      yaw_now = mpu.getYaw();
      yaw_difference = yaw_now - yaw_prev;
      yaw_prev = yaw_now;
      Serial.print("Yaw_difference\t");
      Serial.println(yaw_difference);
      Serial.println("here ok");
      if((fabs(yaw_difference)<0.5) && i>5) 
      {
        Serial.println("yaw<1");
        *initial_yaw = mpu.getYaw();
      }

      lastPrintMillis = currentMillis;
      i++;
    }
  }
}
	
void Setpoint_Adjust_Yaw (float yaw, int motor_num, int motor_direction, double setpoint_m[4] , int optimum_speed_m, const int speed_chg_per_yaw ) 
{
	if(motor_direction == CLOCKWISE)
      {
        setpoint_m[motor_num] = optimum_speed_m + yaw * speed_chg_per_yaw ;
	  }
	  else
	  {
		 setpoint_m[motor_num] = optimum_speed_m - yaw * speed_chg_per_yaw ;
	  }
	  return;
}	  
       