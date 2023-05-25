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

/*define used in sparkfun 9DOF ICM20948 library*/
#define SERIAL_PORT Serial
#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined

// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1
/***********************************************/

/*define used in encoder*/
#define PIN_A_ENC1   35 // Black, cw
#define PIN_B_ENC1   34 // White, ccw
#define PIN_A_ENC2   39 // Black, cw
#define PIN_B_ENC2   36 // White, ccw
/***********************/



ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object


int previous_direction,i;

/**********************LIDAR Parameter**********************/
int dist; /*----actual distance measurements of LiDAR---*/
int strength; /*----signal strength of LiDAR----------------*/
float temprature;
unsigned char check;        /*----save check value------------------------*/
unsigned char uart[9];  /*----save data measured by LiDAR-------------*/
const int HEADER=0x59; /*----frame header of data package------------*/
int rec_debug_state = 0x01;//receive state for frame
/********************************************************/

/*******************Encoder Parameter********************/
float ENC_COUNT_REV = 4800; // Motor encoder output pulses per 360 degree revolution (measured manually)
int counter_enc1 = 0;
int counter_enc2 = 0;
float rpm_enc1 = 0;
float rpm_enc2 = 0;
float radius = 0.025; //in meter

// constant value for calculating velocity
int interval = 1000;
long previousMillis = 0;
long currentMillis = 0;

float ang_velocity = 0;
float ang_velocity_deg = 0;

const float rpm_to_radians = 0.10481975512;
const float rad_to_deg = 57.28578;
const float twopi_r = 0.15707963267948966192;

float distance_enc1 = 0;   
float distance_enc2 = 0;  
/********************************************************/

/*********Bias Storing for ICM20948 Parameter*************/
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
  //SERIAL_PORT.print(F("Gyro X: "));
  //SERIAL_PORT.print(store->biasGyroX);
  //SERIAL_PORT.print(F(" Gyro Y: "));
  //SERIAL_PORT.print(store->biasGyroY);
  //SERIAL_PORT.print(F(" Gyro Z: "));
  //SERIAL_PORT.println(store->biasGyroZ);
  //SERIAL_PORT.print(F("Accel X: "));
  //SERIAL_PORT.print(store->biasAccelX);
  //SERIAL_PORT.print(F(" Accel Y: "));
  //SERIAL_PORT.print(store->biasAccelY);
  //SERIAL_PORT.print(F(" Accel Z: "));
  //SERIAL_PORT.println(store->biasAccelZ);
  //SERIAL_PORT.print(F("CPass X: "));
  //SERIAL_PORT.print(store->biasCPassX);
  //SERIAL_PORT.print(F(" CPass Y: "));
  //SERIAL_PORT.print(store->biasCPassY);
  //SERIAL_PORT.print(F(" CPass Z: "));
  //SERIAL_PORT.println(store->biasCPassZ);

}
/**********************************************/



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
      //SERIAL_PORT.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  //SERIAL_PORT.println(F("Device connected!"));

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
    //SERIAL_PORT.println(F("DMP enabled!"));
#endif
  }
  else
  {
    //SERIAL_PORT.println(F("Enable DMP failed!"));
    //SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }
  
  if (!EEPROM.begin(128)) // Allocate 128 Bytes for EEPROM storage. ESP32 needs this.
  {
    //SERIAL_PORT.println(F("EEPROM.begin failed! You will not be able to save the biases..."));
  }

  biasStore store;

  EEPROM.get(0, store); // Read existing EEPROM, starting at address 0
  if (isBiasStoreValid(&store))
  {
    //SERIAL_PORT.println(F("Bias data in EEPROM is valid. Restoring it..."));
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
      //SERIAL_PORT.println(F("Biases restored."));
      printBiases(&store);
    }
    // else
      // SERIAL_PORT.println(F("Bias restore failed!"));
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
    ////SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) //SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) //SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) //SERIAL_PORT.print( "0" );
    ////SERIAL_PORT.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      ////SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

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
	  
      // //SERIAL_PORT.print(F("Roll:"));
      // //SERIAL_PORT.print(roll, 1);
      // //SERIAL_PORT.print(F(" Pitch:"));
      // //SERIAL_PORT.print(pitch, 1);
      // //SERIAL_PORT.print(F(" Yaw:"));
      // //SERIAL_PORT.println(yaw, 1);
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
	//Serial.println("getting initial yaw");
	while ( *initial_yaw == 0)
	{
      ICM20948_GET_READING_QUAT6(&yaw_now);
      yaw_difference = yaw_now - yaw_prev;
      yaw_prev = yaw_now;
      //Serial.print("Yaw_difference\t");
      //Serial.println(yaw_difference);
      //Serial.println("here ok");
      if(fabs(yaw_difference)<0.01)
      {
        //Serial.println("yaw<0.01");
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
	//Serial.print("Angle now:");
    //Serial.println(*angle);
	return;
}



void calc_robot_dir ( float *magnitude , float *angle, int setpoint_m[4] , const int optimum_speed_m[4],int motor_direction[4],int dji_speed_increase)
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
		setpoint_m[0] = optimum_speed_m[0]+12+dji_speed_increase;
		setpoint_m[1] = optimum_speed_m[1]+12+dji_speed_increase;
		setpoint_m[2] = optimum_speed_m[2]-12-dji_speed_increase;
		setpoint_m[3] = optimum_speed_m[3]-12-dji_speed_increase;
		
	}
	
	else if(*angle>30 && *angle <=60)
	{
		// *robot_dir_now = UPWARD_RIGHT;
		ratio = *angle / 90;
		setpoint_m[0] = optimum_speed_m[0]+12+dji_speed_increase;
		setpoint_m[1] = optimum_speed_m[1];
		setpoint_m[2] = optimum_speed_m[2]-12-dji_speed_increase;
		setpoint_m[3] = optimum_speed_m[3];
	}
	
	else if(*angle>60 && *angle <=90)
	{
		// *robot_dir_now = FORWARD;
		ratio = *angle / 90;
		setpoint_m[0] = optimum_speed_m[0]+12+dji_speed_increase;
		setpoint_m[1] = optimum_speed_m[1]-12-dji_speed_increase;
		setpoint_m[2] = optimum_speed_m[2]-12-dji_speed_increase;
		setpoint_m[3] = optimum_speed_m[3]+12+dji_speed_increase;
	}
	
	else if(*angle>90 && *angle <=120)
	{
		// *robot_dir_now = FORWARD;
		ratio = (*angle-90) / 90;
		setpoint_m[0] = optimum_speed_m[0]+12+dji_speed_increase;
		setpoint_m[1] = optimum_speed_m[1]-12-dji_speed_increase;
		setpoint_m[2] = optimum_speed_m[2]-12-dji_speed_increase;
		setpoint_m[3] = optimum_speed_m[3]+12+dji_speed_increase;
	}
	
	else if(*angle>120 && *angle <=150)
	{
		// *robot_dir_now = UPWARD_LEFT;
		setpoint_m[0] = optimum_speed_m[0];
		setpoint_m[1] = optimum_speed_m[1]-12-dji_speed_increase;
		setpoint_m[2] = optimum_speed_m[2];
		setpoint_m[3] = optimum_speed_m[3]+12+dji_speed_increase;
	}
	
	else if(*angle>150 && *angle <=180)
	{
		// *robot_dir_now = LEFTWARD;
		ratio = (*angle-90) / 90;
		setpoint_m[0] = optimum_speed_m[0]-12-dji_speed_increase;
		setpoint_m[1] = optimum_speed_m[1]-12-dji_speed_increase;
		setpoint_m[2] = optimum_speed_m[2]+12+dji_speed_increase;
		setpoint_m[3] = optimum_speed_m[3]+12+dji_speed_increase;
	}
	
	else if(*angle>-180 && *angle <=-150)
	{
		// *robot_dir_now = LEFTWARD;
		ratio = (*angle+180) / 90;
		setpoint_m[0] = optimum_speed_m[0]-12-dji_speed_increase;
		setpoint_m[1] = optimum_speed_m[1]-12-dji_speed_increase;
		setpoint_m[2] = optimum_speed_m[2]+12+dji_speed_increase;
		setpoint_m[3] = optimum_speed_m[3]+12+dji_speed_increase;
	}
	
	else if(*angle>-150 && *angle <=-120)
	{
		// *robot_dir_now = DOWNWARD_LEFT;
		ratio = (*angle+180) / 90;
		setpoint_m[0] = optimum_speed_m[0]-12-dji_speed_increase;
		setpoint_m[1] = optimum_speed_m[1];
		setpoint_m[2] = optimum_speed_m[2]+12+dji_speed_increase;
		setpoint_m[3] = optimum_speed_m[3];
	}
	
	else if(*angle>-120 && *angle <=-90)
	{
		// *robot_dir_now = BACKWARD;
		ratio = (*angle+180) / 90;
		setpoint_m[0] = optimum_speed_m[0]-12-dji_speed_increase;
		setpoint_m[1] = optimum_speed_m[1]+12+dji_speed_increase;
		setpoint_m[2] = optimum_speed_m[2]+12+dji_speed_increase;
		setpoint_m[3] = optimum_speed_m[3]-12-dji_speed_increase;
	}
	
	else if(*angle>-90 && *angle <=-60)
	{
		// *robot_dir_now = BACKWARD;
		ratio = -(*angle) / 90;
		setpoint_m[0] = optimum_speed_m[0]-12-dji_speed_increase;
		setpoint_m[1] = optimum_speed_m[1]+12+dji_speed_increase;
		setpoint_m[2] = optimum_speed_m[2]+12+dji_speed_increase;
		setpoint_m[3] = optimum_speed_m[3]-12-dji_speed_increase;
	}
	
	else if(*angle>-60 && *angle <=-30)
	{
		// *robot_dir_now = DOWNWARD_RIGHT;
		ratio = -(*angle) / 90;
		setpoint_m[0] = optimum_speed_m[0];
		setpoint_m[1] = optimum_speed_m[1]+12+dji_speed_increase;
		setpoint_m[2] = optimum_speed_m[2];
		setpoint_m[3] = optimum_speed_m[3]-12-dji_speed_increase;
	}
	
	else if(*angle>-30 && *angle <=0)
	{
		// *robot_dir_now = RIGHTWARD;
		ratio = -(*angle) / 90;
		setpoint_m[0] = optimum_speed_m[0]+12+dji_speed_increase;
		setpoint_m[1] = optimum_speed_m[1]+12+dji_speed_increase;
		setpoint_m[2] = optimum_speed_m[2]-12-dji_speed_increase;
		setpoint_m[3] = optimum_speed_m[3]-12-dji_speed_increase;
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
		
/*
int Get_Lidar_data(){
if (Serial.available()) //check if serial port has data input
    {
    if(rec_debug_state == 0x01)
        {  //the first byte
          uart[0]=Serial.read();
          if(uart[0] == 0x59)
              {
                check = uart[0];
                rec_debug_state = 0x02;
              }
        }
else if(rec_debug_state == 0x02)
     {//the second byte
      uart[1]=Serial.read();
      if(uart[1] == 0x59)
          {
            check += uart[1];
            rec_debug_state = 0x03;
          }
      else{
            rec_debug_state = 0x01;
          }
      }

else if(rec_debug_state == 0x03)
        {
          uart[2]=Serial.read();
          check += uart[2];
          rec_debug_state = 0x04;
        }
else if(rec_debug_state == 0x04)
        {
          uart[3]=Serial.read();
          check += uart[3];
          rec_debug_state = 0x05;
        }
else if(rec_debug_state == 0x05)
        {
          uart[4]=Serial.read();
          check += uart[4];
          rec_debug_state = 0x06;
        }
else if(rec_debug_state == 0x06)
        {
          uart[5]=Serial.read();
          check += uart[5];
          rec_debug_state = 0x07;
        }
else if(rec_debug_state == 0x07)
        {
          uart[6]=Serial.read();
          check += uart[6];
          rec_debug_state = 0x08;
        }
else if(rec_debug_state == 0x08)
        {
          uart[7]=Serial.read();
          check += uart[7];
          rec_debug_state = 0x09;
        }
else if(rec_debug_state == 0x09)
        {
          uart[8]=Serial.read();
          if(uart[8] == check)
            {
              
              dist = uart[2] + uart[3]*256;//the distance
              strength = uart[4] + uart[5]*256;//the strength
              temprature = uart[6] + uart[7] *256;//calculate chip temprature
              temprature = temprature/8 - 256;                              
              // Serial.print("dist = ");
              // Serial.print(dist); //output measure distance value of LiDAR
              // Serial.print('\n');
              //Serial.print("strength = ");
              //Serial.print(strength); //output signal strength value
              //Serial.print('\n');
              //Serial.print("\t Chip Temprature = ");
              //Serial.print(temprature);
              //Serial.println(" celcius degree"); //output chip temperature of Lidar                                                       
                while(Serial.available()){Serial.read();} // This part is added becuase some previous packets are there in the buffer so to clear serial buffer and get fresh data.
              delay(150);

              
              // distdist = (distcheck2 - distcheck);
              // distdist2 = ( distcheck - distcheck2);
              // distcheck3 = distcheck2;
              // distcheck2 = distcheck;
              // distcheck = dist ; 

//              distdist = abs(distcheck2 - dist);
//              distcheck = dist ; 
//              distcheck2 = distcheck;
//              
//               if (flag==1)
//              {
//                if (distdist>100)
//                {
//                digitalWrite(13,LOW); //LED LOW
//                flag = 0;
//                }
//              }
              // if (distdist2>270)
              // {
                // digitalWrite(13,LOW); //LED LOW
                // Serial.print('\n');
                // Serial.println(" LED OFF ");
                // flag = 0;
              // }
              // if (distdist>270)
              // {
               // if (distdist>270)
                // {
                // distdist = (distcheck3 - distcheck);
                // if (flag == 0 ) 
                // {
                  
                  // Serial.print('\n');
                  // Serial.print("diff = ");
                  // Serial.print(distdist); //D1 & D2 diffence
                  // Serial.print('\n');
                  // Serial.print("last distance = ");
                  // Serial.print(distcheck3); //Distance
                  // Serial.print('\n');
                  // Serial.println(distcheck); //Distance
                  // digitalWrite(13,HIGH); // LED
                  //Serial.print('\n');
                  // Serial.println(" LED ON ");
                  // x=65/dist;
                  // bx = atan2 (65,dist) * 180/3.14159265 ; // radians to degrees and rounding
                  // Serial.print("x = ");
                  // Serial.print(x); //output x value
                  // Serial.print('\n'); 
                  // Serial.print("angle = ");
                  // Serial.print(bx); //output angle
                  // Serial.println('\n'); 
                  // flag=1; 
             
              
              

             }
          rec_debug_state = 0x01;
        }
    }
	
	return dist;

}
*/

void CalculateVelocity()
    {
        currentMillis = millis();

        if( currentMillis - previousMillis > interval )
        {
            previousMillis = currentMillis;
            rpm_enc1 = float (counter_enc1 * 60 / ENC_COUNT_REV) ;
            rpm_enc2 = float (counter_enc2 * 60 / ENC_COUNT_REV) ;
//            ang_velocity = rpm * rpm_to_radians;
//            ang_velocity_deg = ang_velocity * rad_to_deg;
            distance_enc1 += (rpm_enc1 / 60 * twopi_r);
            distance_enc2 += (rpm_enc2 / 60 * twopi_r);
//
//            Serial.print("Pulse for encoder 1: ");
//            Serial.println( counter_enc1 );
//            Serial.print("Pulse for encoder 2: ");
//            Serial.println( counter_enc2 );
//            Serial.print("Speed for encoder 1: ");
//            Serial.println(rpm_enc1,4);
//            Serial.print("Speed for encoder 2: ");
//            Serial.println(rpm_enc2,4);
            
//            Serial.println("RPM ");
//            Serial.print("Angular Velocity : ");
//            Serial.print( ang_velocity_deg,4 );
//            Serial.println(" deg per second");

//            Serial.print( "Distance for encoder 1: " );
//            Serial.println( distance_enc1,4 );
//            Serial.print( "Distance for encoder 2: " );
//            Serial.println( distance_enc2,4 );
//            Serial.println();

            counter_enc1 = 0;
            counter_enc2 = 0;
        }        
    }

void checkEncoderA1() 
    {
      // digitalRead - reads the value from a specified pin, either HIGH or LOW.
      int pinA_enc1 = digitalRead(PIN_A_ENC1);
      int pinB_enc1 = digitalRead(PIN_B_ENC1);

      if ( pinA_enc1 == pinB_enc1 )
      {
        counter_enc1++;
      } 
      else 
      {
        counter_enc1--;
      }
    }

void checkEncoderB1() 
    {
      // digitalRead - reads the value from a specified pin, either HIGH or LOW.
      int pinA_enc1 = digitalRead(PIN_A_ENC1);
      int pinB_enc1 = digitalRead(PIN_B_ENC1);
      if ( pinA_enc1 != pinB_enc1 )
      {
        counter_enc1++;
      } 
      else 
      {
        counter_enc1--;
      }
    }

void checkEncoderA2() 
    {
      // digitalRead - reads the value from a specified pin, either HIGH or LOW.
      int pinA_enc2 = digitalRead(PIN_A_ENC2);
      int pinB_enc2 = digitalRead(PIN_B_ENC2);

      if ( pinA_enc2 == pinB_enc2 )
      {
        counter_enc2++;
      } 
      else 
      {
        counter_enc2--;
      }
    }

void checkEncoderB2() 
    {
      // digitalRead - reads the value from a specified pin, either HIGH or LOW.
      int pinA_enc2 = digitalRead(PIN_A_ENC2);
      int pinB_enc2 = digitalRead(PIN_B_ENC2);
      if ( pinA_enc2 != pinB_enc2 )
      {
        counter_enc2++;
      } 
      else 
      {
        counter_enc2--;
      }
    }
	
void Encoder_Calc ( float distance_x, float distance_y, float *magnitude, float *angle )
{
	const float pi = 3.141592653589793238;
	//Serial.print("distance_x now:");
	//Serial.println(distance_x);
	//Serial.print("distance_y now:");
	//Serial.println(distance_y);

	*magnitude = std::sqrt(distance_x*distance_x + distance_y*distance_y); // calculate the magnitude using the Pythagorean theorem
    *angle = std::atan2(distance_y, distance_x) * 180 / pi; // calculate the angle using the arctangent function, and convert from radians to degrees
	return;
}

// void Encoder_Positioning( target_x_distance,target_y_distance,max_encoder_positioning_error)
// {
	// int dji_spd_chg = -3;
      // while(fabs(distance_enc1-TARGET_Y_DISTANCE)>max_encoder_positioning_error)
      // {
        // CalculateVelocity();
        // Serial.print("distance y encoder:");
        // Serial.println(distance_enc1);
      // if(distance_enc1<TARGET_Y_DISTANCE)
      // {
        // Angle_L=90;
        // Magnitude_L=50;
        // calc_robot_dir ( &Magnitude_L , &Angle_L, pulse_M , default_M,Motor_dir,DJI_Increase_Speed);
        // for(i=0;i<4;i++)
        // {
        // Setpoint_Adjust_Yaw (Yaw_difference, i,Motor_dir[i], pulse_M , SPEED_CHG_PER_YAW );
        // Serial.print("Setpoint of Motor:");
        // Serial.print(i);
        // Serial.print("is: ");
        // Serial.println (pulse_M[i]);
        // }
        // for(i = 0; i < 4; i++)
        // {
          // ledcWrite(i+3,pulse_M[i]);
        // }
      // }
      // else if(distance_enc1>TARGET_Y_DISTANCE)
      // {
        // Angle_L=-89;
        // Magnitude_L=50;
        // calc_robot_dir ( &Magnitude_L , &Angle_L, pulse_M , default_M,Motor_dir,DJI_Increase_Speed);
        // for(i=0;i<4;i++)
        // {
        // Setpoint_Adjust_Yaw (Yaw_difference, i,Motor_dir[i], pulse_M , SPEED_CHG_PER_YAW );
        // Serial.print("Setpoint of Motor:");
        // Serial.print(i);
        // Serial.print("is: ");
        // Serial.println (pulse_M[i]);
        // }
        // for(i = 0; i < 4; i++)
        // {
          // ledcWrite(i+3,pulse_M[i]);
        // }
      // }
      // }
// }