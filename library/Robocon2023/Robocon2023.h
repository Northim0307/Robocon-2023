//Robot Direction
#define STOP 0
#define FORWARD 1
#define BACKWARD 2
#define LEFTWARD 3
#define RIGHTWARD 4
#define ROTATE_CW 5 
#define ROTATE_C_CW 6
#define UPWARD_LEFT 7
#define UPWARD_RIGHT 8
#define DOWNWARD_LEFT 9
#define DOWNWARD_RIGHT 10

#define MOVING 1
#define STOPPING 0

#define CLOCKWISE 0
#define COUNTER_CLOCKWISE 1
#define HALT 2

#define STANDARD_CAN_FRAME 0

//MPU9250
#define MPU9250_IMU_ADDRESS 0x68
#define INTERVAL_MS_PRINT 1000

#define SERIAL_PORT Serial

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#include <EEPROM.h>

// Define a storage struct for the biases. Include a non-zero header and a simple checksum
struct biasStore
{
  int32_t header = 0x42;
  int32_t biasGyroX = 0;
  int32_t biasGyroY = 0;
  int32_t biasGyroZ = 0;
  int32_t biasAccelX = 0;
  int32_t biasAccelY = 0;
  int32_t biasAccelZ = 0;
  int32_t biasCPassX = 0;
  int32_t biasCPassY = 0;
  int32_t biasCPassZ = 0;
  int32_t sum = 0;
};

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void updateBiasStoreSum(biasStore *store) // Update the bias store checksum
{
  int32_t sum = store->header;
  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;
  store->sum = sum;
}

bool isBiasStoreValid(biasStore *store) // Returns true if the header and checksum are valid
{
  int32_t sum = store->header;

  if (sum != 0x42)
    return false;

  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;

  return (store->sum == sum);
}

void printBiases(biasStore *store)
{
  SERIAL_PORT.print(F("Gyro X: "));
  SERIAL_PORT.print(store->biasGyroX);
  SERIAL_PORT.print(F(" Gyro Y: "));
  SERIAL_PORT.print(store->biasGyroY);
  SERIAL_PORT.print(F(" Gyro Z: "));
  SERIAL_PORT.println(store->biasGyroZ);
  SERIAL_PORT.print(F("Accel X: "));
  SERIAL_PORT.print(store->biasAccelX);
  SERIAL_PORT.print(F(" Accel Y: "));
  SERIAL_PORT.print(store->biasAccelY);
  SERIAL_PORT.print(F(" Accel Z: "));
  SERIAL_PORT.println(store->biasAccelZ);
  SERIAL_PORT.print(F("CPass X: "));
  SERIAL_PORT.print(store->biasCPassX);
  SERIAL_PORT.print(F(" CPass Y: "));
  SERIAL_PORT.print(store->biasCPassY);
  SERIAL_PORT.print(F(" CPass Z: "));
  SERIAL_PORT.println(store->biasCPassZ);

}


ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object


// struct can_frame {
    // unsigned long   can_id;       // CAN_ID 
    // byte            can_dlc;      // frame data length in byte 
    // byte            data[8]={0};  // CAN_DATA 
// }
// can_output,         //CAN struct to store id,dlc and data for output from MCU to Speed controller
// can_input,          //CAN struct to store id,dlc and data for input  from Speed controller to MCU
// can_motor_msg[4];   //CAN struct to store id,dlc and data for M1 and M2

int previous_direction,i;

// MCP_CAN CAN0(5);     // Set CS pin to pin 5

// MPU9250 mpu;


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
/*
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
*/

void ICM20948_SETUP()
{
	  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);



  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL);


    if (myICM.status != ICM_20948_Stat_Ok)
    {

      SERIAL_PORT.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }


  SERIAL_PORT.println(F("Device connected!"));

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP orientation sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
#ifndef QUAT_ANIMATION
    SERIAL_PORT.println(F("DMP enabled!"));
#endif
  }
  else
  {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }
  
  if (!EEPROM.begin(128)) // Allocate 128 Bytes for EEPROM storage. ESP32 needs this.
  {
    SERIAL_PORT.println(F("EEPROM.begin failed! You will not be able to save the biases..."));
  }

  biasStore store;

  EEPROM.get(0, store); // Read existing EEPROM, starting at address 0
  if (isBiasStoreValid(&store))
  {
    SERIAL_PORT.println(F("Bias data in EEPROM is valid. Restoring it..."));
    success &= (myICM.setBiasGyroX(store.biasGyroX) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasGyroY(store.biasGyroY) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasGyroZ(store.biasGyroZ) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasAccelX(store.biasAccelX) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasAccelY(store.biasAccelY) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasAccelZ(store.biasAccelZ) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasCPassX(store.biasCPassX) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasCPassY(store.biasCPassY) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasCPassZ(store.biasCPassZ) == ICM_20948_Stat_Ok);

    if (success)
    {
      SERIAL_PORT.println(F("Biases restored."));
      printBiases(&store);
    }
    else
      SERIAL_PORT.println(F("Bias restore failed!"));
  }



  return;
}

void ICM20948_GET_READING(double *yaw)
{
	double roll,pitch;

	// Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);
  

  


  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    while(myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)
    {
      myICM.readDMPdataFromFIFO(&data);
    }
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);

      // Scale to +/- 1
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double q2sqr = q2 * q2;

      // roll (x-axis rotation)
      double t0 = +2.0 * (q0 * q1 + q2 * q3);
      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      *yaw = atan2(t3, t4) * 180.0 / PI;
	  
      // SERIAL_PORT.print(F("Roll:"));
      // SERIAL_PORT.print(roll, 1);
      // SERIAL_PORT.print(F(" Pitch:"));
      // SERIAL_PORT.print(pitch, 1);
      // SERIAL_PORT.print(F(" Yaw:"));
      // SERIAL_PORT.println(yaw, 1);
    }
  }

  
  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
  
  return ;
}	

// void getInitialYaw(float *initial_yaw)
 void getInitialYaw(double *initial_yaw)
{
	double yaw_difference,yaw_now,yaw_prev;
	unsigned long currentMillis = 0, lastPrintMillis = 0;
	Serial.println("getting initial yaw");
	while ( *initial_yaw == 0)
	{
      ICM20948_GET_READING(&yaw_now);
      yaw_difference = yaw_now - yaw_prev;
      yaw_prev = yaw_now;
      Serial.print("Yaw_difference\t");
      Serial.println(yaw_difference);
      Serial.println("here ok");
      if(fabs(yaw_difference)<0.01)
      {
        Serial.println("yaw<0.01");
        // *initial_yaw = mpu.getYaw();
		 *initial_yaw = yaw_now;
      }    
	}
}
	
void Setpoint_Adjust_Yaw (double yaw, int motor_num, int motor_direction, int setpoint_m[4], const int speed_chg_per_yaw ) 
{
	/*use to limit the speed change due to yaw*/
	if(yaw>20)
	{
		yaw=20;
	}
	else if(yaw<-20)
	{
		yaw=20;
	}
	
		if(motor_direction == CLOCKWISE)
		{
			setpoint_m[motor_num] = setpoint_m[motor_num] - (yaw * speed_chg_per_yaw)/3;
		}
		else if(motor_direction == COUNTER_CLOCKWISE)
		{
			setpoint_m[motor_num] = setpoint_m[motor_num] + (yaw * speed_chg_per_yaw)/3 ;
		}
	return;
}	  
       
void Analog_Stick_Calc ( float x, float y, float *magnitude, float *angle )
{
	const float pi = 3.141592653589793238;
	// Serial.print("x now:");
	// Serial.println(x);
	// Serial.print("y now:");
	// Serial.println(y);
	if(fabs(x)<5) x = 0;
	if(fabs(y)<5) y = 0;

	*magnitude = std::sqrt(x*x + y*y); // calculate the magnitude using the Pythagorean theorem
    *angle = std::atan2(y, x) * 180 / pi; // calculate the angle using the arctangent function, and convert from radians to degrees
	Serial.print("Angle now:");
    Serial.println(*angle);
	return;
}

// void calc_robot_dir ( int *robot_dir_now, float *magnitude , float *angle, int motor_direction[4], double setpoint_m[4] , const int optimum_speed_m[4])
// {
	// float ratio_speed_m[4];
	// float ratio;
	
	// Serial.print("magnitude now:");
	// Serial.println(*magnitude);
	// Serial.print("angle now:");
	// Serial.println(*angle);
	
	// float magnitude_ratio = 1;
	//// float magnitude_ratio = *magnitude / 127 ;
	//// if(magnitude_ratio>1) magnitude_ratio = 1;
	
	// Serial.print("magnitude_ratio now:");
	// Serial.println(magnitude_ratio);
	
	// for(int k = 0; k<4;k++)
	// {
		// ratio_speed_m[k] = optimum_speed_m[k] * magnitude_ratio;

	// }

// if(*magnitude <30 )
// {
	// *robot_dir_now = STOP;
		// return;
// }
	
	//// if(ratio_speed_m[0] < 10)
	//// {
		//// *robot_dir_now = STOP;
		//// return;
	//// }
	
	// if(*angle>0 && *angle <=90)
	// {
		// *robot_dir_now = UPWARD_RIGHT;
		// ratio = *angle / 90;
		// setpoint_m[0] = -(ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0]);
		// setpoint_m[1] = ratio * ratio_speed_m[0] - (1-ratio) * ratio_speed_m[0];
		// setpoint_m[2] = - setpoint_m[0];
		// setpoint_m[3] = - setpoint_m[1];
	// }
	// else if(*angle>90 && *angle <=180)
	// {
		// *robot_dir_now = UPWARD_LEFT;
		// ratio = (*angle-90) / 90;
		// setpoint_m[0] = (ratio * ratio_speed_m[0]) - (1-ratio) * ratio_speed_m[0];
		// setpoint_m[1] =   ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0];
		// setpoint_m[2] = - setpoint_m[0];
		// setpoint_m[3] = - setpoint_m[1];
	// }
	// else if(*angle>-180 && *angle <=-90)
	// {
		// *robot_dir_now = DOWNWARD_LEFT;
		// ratio = (*angle+180) / 90;
		// setpoint_m[0] = ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0];
		// setpoint_m[1] = -(ratio * ratio_speed_m[0]) + (1-ratio) * ratio_speed_m[0];
		// setpoint_m[2] = - setpoint_m[0];
		// setpoint_m[3] = - setpoint_m[1];
	// }
	// else if(*angle>-90 && *angle <=0)
	// {
		// *robot_dir_now = DOWNWARD_RIGHT;
		// ratio = -(*angle) / 90;
		// setpoint_m[0] = ratio * ratio_speed_m[0] - (1-ratio) * ratio_speed_m[0];
		// setpoint_m[1] = -(ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0]);
		// setpoint_m[2] = - setpoint_m[0];
		// setpoint_m[3] = - setpoint_m[1];
	// }
	
	// for(int k = 0; k < 4;k++)
	// {
		// if(setpoint_m[k]<0)
		// {
			// motor_direction[k] = COUNTER_CLOCKWISE;
			// setpoint_m[k] = - setpoint_m[k];
		// }
		// else
		// {
			// motor_direction[k] = CLOCKWISE;
		// }
	// }
	
	// for(int k = 0; k < 4;k++)
	// {
		// Serial.print("Setpoint ");
		// Serial.print("Motor ");
		// Serial.print(k);
		// Serial.print(" :");
		// Serial.println(setpoint_m[k]);
		// Serial.print("Motor Direction of M");
		// Serial.print(k);
		// Serial.print(" :");
		// if(motor_direction[k] == COUNTER_CLOCKWISE)
		// {
			// Serial.println(" COUNTER_CLOCKWISE");
		// }
		// else
		// {
			// Serial.println(" CCLOCKWISE");
		// }
	// }
	
	// return;
// }

// void calc_robot_dir ( int *robot_dir_now, float *magnitude , float *angle, int motor_direction[4], double setpoint_m[4] , const int optimum_speed_m[4])
// {
	// float ratio_speed_m[4];
	// float ratio;
	
	// Serial.print("magnitude now:");
	// Serial.println(*magnitude);
	// Serial.print("angle now:");
	// Serial.println(*angle);
	
	// float magnitude_ratio = 1;
	//// float magnitude_ratio = *magnitude / 127 ;
	//// if(magnitude_ratio>1) magnitude_ratio = 1;
	
	// Serial.print("magnitude_ratio now:");
	// Serial.println(magnitude_ratio);
	
	// for(int k = 0; k<4;k++)
	// {
		// ratio_speed_m[k] = optimum_speed_m[k] * magnitude_ratio;

	// }

// if(*magnitude <30 )
// {
	// *robot_dir_now = STOP;
		// return;
// }
	
	//// if(ratio_speed_m[0] < 10)
	//// {
		//// *robot_dir_now = STOP;
		//// return;
	//// }
	// if(*angle>0 && *angle <=30)
	// {
		// *robot_dir_now = RIGHTWARD;
		// ratio = *angle / 90;
		// setpoint_m[0] = -(ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0]);
		// setpoint_m[1] = ratio * ratio_speed_m[0] - (1-ratio) * ratio_speed_m[0];
		// setpoint_m[2] = - setpoint_m[0];
		// setpoint_m[3] = - setpoint_m[1];
		
	// }
	
	// else if(*angle>30 && *angle <=60)
	// {
		// *robot_dir_now = UPWARD_RIGHT;
		// ratio = *angle / 90;
		// setpoint_m[0] = -(ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0]);
		// setpoint_m[1] = 0;
		// setpoint_m[2] = - setpoint_m[0];
		// setpoint_m[3] = 0;
	// }
	
	// else if(*angle>60 && *angle <=90)
	// {
		// *robot_dir_now = FORWARD;
		// ratio = *angle / 90;
		// setpoint_m[0] = -(ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0]);
		// setpoint_m[1] = ratio * ratio_speed_m[0] - (1-ratio) * ratio_speed_m[0];
		// setpoint_m[2] = - setpoint_m[0];
		// setpoint_m[3] = - setpoint_m[1];
	// }
	
	// else if(*angle>90 && *angle <=120)
	// {
		// *robot_dir_now = FORWARD;
		// ratio = (*angle-90) / 90;
		// setpoint_m[0] = (ratio * ratio_speed_m[0]) - (1-ratio) * ratio_speed_m[0];
		// setpoint_m[1] =   ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0];
		// setpoint_m[2] = - setpoint_m[0];
		// setpoint_m[3] = - setpoint_m[1];
	// }
	
	// else if(*angle>120 && *angle <=150)
	// {
		// *robot_dir_now = UPWARD_LEFT;
		// ratio = (*angle-90) / 90;
		// setpoint_m[0] = 0;
		// setpoint_m[1] =   ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0];
		// setpoint_m[2] = 0;
		// setpoint_m[3] = - setpoint_m[1];
	// }
	
	// else if(*angle>150 && *angle <=180)
	// {
		// *robot_dir_now = LEFTWARD;
		// ratio = (*angle-90) / 90;
		// setpoint_m[0] = (ratio * ratio_speed_m[0]) - (1-ratio) * ratio_speed_m[0];
		// setpoint_m[1] =   ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0];
		// setpoint_m[2] = - setpoint_m[0];
		// setpoint_m[3] = - setpoint_m[1];
	// }
	
	// else if(*angle>-180 && *angle <=-150)
	// {
		// *robot_dir_now = LEFTWARD;
		// ratio = (*angle+180) / 90;
		// setpoint_m[0] = ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0];
		// setpoint_m[1] = -(ratio * ratio_speed_m[0]) + (1-ratio) * ratio_speed_m[0];
		// setpoint_m[2] = - setpoint_m[0];
		// setpoint_m[3] = - setpoint_m[1];
	// }
	
	// else if(*angle>-150 && *angle <=-120)
	// {
		// *robot_dir_now = DOWNWARD_LEFT;
		// ratio = (*angle+180) / 90;
		// setpoint_m[0] = ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0];
		// setpoint_m[1] = 0;
		// setpoint_m[2] = - setpoint_m[0];
		// setpoint_m[3] = 0;
	// }
	
	// else if(*angle>-120 && *angle <=-90)
	// {
		// *robot_dir_now = BACKWARD;
		// ratio = (*angle+180) / 90;
		// setpoint_m[0] = ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0];
		// setpoint_m[1] = -(ratio * ratio_speed_m[0]) + (1-ratio) * ratio_speed_m[0];
		// setpoint_m[2] = - setpoint_m[0];
		// setpoint_m[3] = - setpoint_m[1];
	// }
	
	// else if(*angle>-90 && *angle <=-60)
	// {
		// *robot_dir_now = BACKWARD;
		// ratio = -(*angle) / 90;
		// setpoint_m[0] = ratio * ratio_speed_m[0] - (1-ratio) * ratio_speed_m[0];
		// setpoint_m[1] = -(ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0]);
		// setpoint_m[2] = - setpoint_m[0];
		// setpoint_m[3] = - setpoint_m[1];
	// }
	
	// else if(*angle>-60 && *angle <=-30)
	// {
		// *robot_dir_now = DOWNWARD_RIGHT;
		// ratio = -(*angle) / 90;
		// setpoint_m[0] = 0;
		// setpoint_m[1] = -(ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0]);
		// setpoint_m[2] = 0;
		// setpoint_m[3] = - setpoint_m[1];
	// }
	
	// else if(*angle>-30 && *angle <=0)
	// {
		// *robot_dir_now = RIGHTWARD;
		// ratio = -(*angle) / 90;
		// setpoint_m[0] = ratio * ratio_speed_m[0] - (1-ratio) * ratio_speed_m[0];
		// setpoint_m[1] = -(ratio * ratio_speed_m[0] + (1-ratio) * ratio_speed_m[0]);
		// setpoint_m[2] = - setpoint_m[0];
		// setpoint_m[3] = - setpoint_m[1];
	// }
	
	// for(int k = 0; k < 4;k++)
	// {
		// if(setpoint_m[k]<0)
		// {
			// motor_direction[k] = COUNTER_CLOCKWISE;
			// setpoint_m[k] = - setpoint_m[k];
		// }
		// else
		// {
			// motor_direction[k] = CLOCKWISE;
		// }
	// }
	
	// for(int k = 0; k < 4;k++)
	// {
		// Serial.print("Setpoint ");
		// Serial.print("Motor ");
		// Serial.print(k);
		// Serial.print(" :");
		// Serial.println(setpoint_m[k]);
		// Serial.print("Motor Direction of M");
		// Serial.print(k);
		// Serial.print(" :");
		// if(motor_direction[k] == COUNTER_CLOCKWISE)
		// {
			// Serial.println(" COUNTER_CLOCKWISE");
		// }
		// else
		// {
			// Serial.println(" CCLOCKWISE");
		// }
	// }
	
	// return;
// }

void Encoder_Calc ( float distance_x, float distance_y, float *magnitude, float *angle )
{
	const float pi = 3.141592653589793238;
	Serial.print("distance_x now:");
	Serial.println(distance_x);
	Serial.print("distance_y now:");
	Serial.println(distance_y);

	*magnitude = std::sqrt(distance_x*distance_x + distance_y*distance_y); // calculate the magnitude using the Pythagorean theorem
    *angle = std::atan2(distance_y, distance_x) * 180 / pi; // calculate the angle using the arctangent function, and convert from radians to degrees
	return;
}













void calc_robot_dir ( float *magnitude , float *angle, int setpoint_m[4] , const int optimum_speed_m[4],int motor_direction[4])
{
	float ratio_speed_m[4];
	float ratio;
	
	// Serial.print("magnitude now:");
	// Serial.println(*magnitude);
	// Serial.print("angle now:");
	// Serial.println(*angle);
	
	// float magnitude_ratio = 1;
	// float magnitude_ratio = *magnitude / 127 ;
	// if(magnitude_ratio>1) magnitude_ratio = 1;
	
	// Serial.print("magnitude_ratio now:");
	// Serial.println(magnitude_ratio);
	
	// for(int k = 0; k<4;k++)
	// {
		// ratio_speed_m[k] = optimum_speed_m[k] * magnitude_ratio;

	// }

if(*magnitude <30 )
{
	// *robot_dir_now = STOP;
		return;
}
	
	// if(ratio_speed_m[0] < 10)
	// {
		// *robot_dir_now = STOP;
		// return;
	// }
	if(*angle>0 && *angle <=30)
	{
		// *robot_dir_now = RIGHTWARD;
		ratio = *angle / 90;
		setpoint_m[0] = optimum_speed_m[0]+12;
		setpoint_m[1] = optimum_speed_m[1]+12;
		setpoint_m[2] = optimum_speed_m[2]-12;
		setpoint_m[3] = optimum_speed_m[3]-12;
		
	}
	
	else if(*angle>30 && *angle <=60)
	{
		// *robot_dir_now = UPWARD_RIGHT;
		ratio = *angle / 90;
		setpoint_m[0] = optimum_speed_m[0]+12;
		setpoint_m[1] = optimum_speed_m[1];
		setpoint_m[2] = optimum_speed_m[2]-12;
		setpoint_m[3] = optimum_speed_m[3];
	}
	
	else if(*angle>60 && *angle <=90)
	{
		// *robot_dir_now = FORWARD;
		ratio = *angle / 90;
		setpoint_m[0] = optimum_speed_m[0]+12;
		setpoint_m[1] = optimum_speed_m[1]-12;
		setpoint_m[2] = optimum_speed_m[2]-12;
		setpoint_m[3] = optimum_speed_m[3]+12;
	}
	
	else if(*angle>90 && *angle <=120)
	{
		// *robot_dir_now = FORWARD;
		ratio = (*angle-90) / 90;
		setpoint_m[0] = optimum_speed_m[0]+12;
		setpoint_m[1] = optimum_speed_m[1]-12;
		setpoint_m[2] = optimum_speed_m[2]-12;
		setpoint_m[3] = optimum_speed_m[3]+12;
	}
	
	else if(*angle>120 && *angle <=150)
	{
		// *robot_dir_now = UPWARD_LEFT;
		setpoint_m[0] = optimum_speed_m[0];
		setpoint_m[1] = optimum_speed_m[1]-12;
		setpoint_m[2] = optimum_speed_m[2];
		setpoint_m[3] = optimum_speed_m[3]+12;
	}
	
	else if(*angle>150 && *angle <=180)
	{
		// *robot_dir_now = LEFTWARD;
		ratio = (*angle-90) / 90;
		setpoint_m[0] = optimum_speed_m[0]-12;
		setpoint_m[1] = optimum_speed_m[1]-12;
		setpoint_m[2] = optimum_speed_m[2]+12;
		setpoint_m[3] = optimum_speed_m[3]+12;
	}
	
	else if(*angle>-180 && *angle <=-150)
	{
		// *robot_dir_now = LEFTWARD;
		ratio = (*angle+180) / 90;
		setpoint_m[0] = optimum_speed_m[0]-12;
		setpoint_m[1] = optimum_speed_m[1]-15;
		setpoint_m[2] = optimum_speed_m[2]+12;
		setpoint_m[3] = optimum_speed_m[3]+12;
	}
	
	else if(*angle>-150 && *angle <=-120)
	{
		// *robot_dir_now = DOWNWARD_LEFT;
		ratio = (*angle+180) / 90;
		setpoint_m[0] = optimum_speed_m[0]-12;
		setpoint_m[1] = optimum_speed_m[1];
		setpoint_m[2] = optimum_speed_m[2]+12;
		setpoint_m[3] = optimum_speed_m[3];
	}
	
	else if(*angle>-120 && *angle <=-90)
	{
		// *robot_dir_now = BACKWARD;
		ratio = (*angle+180) / 90;
		setpoint_m[0] = optimum_speed_m[0]-12;
		setpoint_m[1] = optimum_speed_m[1]+12;
		setpoint_m[2] = optimum_speed_m[2]+12;
		setpoint_m[3] = optimum_speed_m[3]-12;
	}
	
	else if(*angle>-90 && *angle <=-60)
	{
		// *robot_dir_now = BACKWARD;
		ratio = -(*angle) / 90;
		setpoint_m[0] = optimum_speed_m[0]-12;
		setpoint_m[1] = optimum_speed_m[1]+12;
		setpoint_m[2] = optimum_speed_m[2]+12;
		setpoint_m[3] = optimum_speed_m[3]-12;
	}
	
	else if(*angle>-60 && *angle <=-30)
	{
		// *robot_dir_now = DOWNWARD_RIGHT;
		ratio = -(*angle) / 90;
		setpoint_m[0] = optimum_speed_m[0];
		setpoint_m[1] = optimum_speed_m[1]+12;
		setpoint_m[2] = optimum_speed_m[2];
		setpoint_m[3] = optimum_speed_m[3]-12;
	}
	
	else if(*angle>-30 && *angle <=0)
	{
		// *robot_dir_now = RIGHTWARD;
		ratio = -(*angle) / 90;
		setpoint_m[0] = optimum_speed_m[0]+12;
		setpoint_m[1] = optimum_speed_m[1]+12;
		setpoint_m[2] = optimum_speed_m[2]-12;
		setpoint_m[3] = optimum_speed_m[3]-12;
	}
	
	for(int k = 0; k < 4;k++)
	{
		if(setpoint_m[k]==0 || setpoint_m[k]==-0)
		{
			motor_direction[k] = HALT;
		}
			
		else if(setpoint_m[k]<0)
		{
			motor_direction[k] = COUNTER_CLOCKWISE;
		}
		else
		{
			motor_direction[k] = CLOCKWISE;
		}
	}
	
	// for(int k = 0; k < 4;k++)
	// {
		// Serial.print("Setpoint ");
		// Serial.print("Motor ");
		// Serial.print(k);
		// Serial.print(" :");
		// Serial.println(setpoint_m[k]);
		// Serial.print("Motor Direction of M");
		// Serial.print(k);
		// Serial.print(" :");
		// if(motor_direction[k] == COUNTER_CLOCKWISE)
		// {
			// Serial.println(" COUNTER_CLOCKWISE");
		// }
		// else
		// {
			// Serial.println(" CCLOCKWISE");
		// }
	// }
	
	return;
}

// void Rotate ( int angle )
// {
	// setpoint_m[0] = optimum_speed_m[0]+12;
	// Difference Yaw = Yaw - initial_yaw
	// angle - difference Yaw 
	// if(>0)
	// {
		// setpoint_m[0] = optimum_speed_m[0]+round((angle - difference Yaw));
		
		
		
void ICM20948_SETUP_QUAT6()
{
	  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);



  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {
    myICM.begin(WIRE_PORT, AD0_VAL);


    if (myICM.status != ICM_20948_Stat_Ok)
    {

      SERIAL_PORT.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }


  SERIAL_PORT.println(F("Device connected!"));

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
#ifndef QUAT_ANIMATION
    SERIAL_PORT.println(F("DMP enabled!"));
#endif
  }
  else
  {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }
  
  if (!EEPROM.begin(128)) // Allocate 128 Bytes for EEPROM storage. ESP32 needs this.
  {
    SERIAL_PORT.println(F("EEPROM.begin failed! You will not be able to save the biases..."));
  }

  biasStore store;

  EEPROM.get(0, store); // Read existing EEPROM, starting at address 0
  if (isBiasStoreValid(&store))
  {
    SERIAL_PORT.println(F("Bias data in EEPROM is valid. Restoring it..."));
    success &= (myICM.setBiasGyroX(store.biasGyroX) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasGyroY(store.biasGyroY) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasGyroZ(store.biasGyroZ) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasAccelX(store.biasAccelX) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasAccelY(store.biasAccelY) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasAccelZ(store.biasAccelZ) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasCPassX(store.biasCPassX) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasCPassY(store.biasCPassY) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasCPassZ(store.biasCPassZ) == ICM_20948_Stat_Ok);

    if (success)
    {
      SERIAL_PORT.println(F("Biases restored."));
      printBiases(&store);
    }
    else
      SERIAL_PORT.println(F("Bias restore failed!"));
  }



  return;
}

void ICM20948_GET_READING_QUAT6(double *yaw)
{
	double roll,pitch;

	// Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);
  

  


  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    while(myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)
    {
      myICM.readDMPdataFromFIFO(&data);
    }
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
	  
      double q2sqr = q2 * q2;

      // roll (x-axis rotation)
      double t0 = +2.0 * (q0 * q1 + q2 * q3);
      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      *yaw = atan2(t3, t4) * 180.0 / PI;
	  
      // SERIAL_PORT.print(F("Roll:"));
      // SERIAL_PORT.print(roll, 1);
      // SERIAL_PORT.print(F(" Pitch:"));
      // SERIAL_PORT.print(pitch, 1);
      // SERIAL_PORT.print(F(" Yaw:"));
      // SERIAL_PORT.println(yaw, 1);
    }
  }

  
  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
  
  return ;
}	


