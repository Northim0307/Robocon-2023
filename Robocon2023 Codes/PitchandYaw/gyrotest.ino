#include <ps5Controller.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <Robocon2023.h>
//#define USE_SPI       // Uncomment this to use SPI

// #define SERIAL_PORT Serial

// #define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
// #define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

// #define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// // The value of the last bit of the I2C address.
// // On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
// #define AD0_VAL 1

// #ifdef USE_SPI
// ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
// #else
// ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
// #endif


#define L_ACTUATOR_PWM_PIN 27
#define L_ACTUATOR_DIR_PIN 26

int l_actuator_pwm_channel = 7;
int frequency_actuator = 500;
int resolution_actuator = 8;
double Initial_Yaw = 0;
double Old_Yaw, Yaw =0, Yaw_difference =0;

double Initial_Pitch=0;
double Old_Pitch, Pitch =0, Pitch_difference =0;
unsigned long current_millis;
bool circlePressed = false; // Variable to track the Circle button press

void setup()
{

//   SERIAL_PORT.begin(115200); // Start the serial console
// #ifndef QUAT_ANIMATION
//   SERIAL_PORT.println(F("ICM-20948 Example"));
// #endif

//   delay(100);

// #ifndef QUAT_ANIMATION
//   while (SERIAL_PORT.available()) // Make sure the serial RX buffer is empty
//     SERIAL_PORT.read();

//   SERIAL_PORT.println(F("Press any key to continue..."));

//   while (!SERIAL_PORT.available()) // Wait for the user to press a key (send any serial character)
//     ;
// #endif

// #ifdef USE_SPI
//   SPI_PORT.begin();
// #else
//   WIRE_PORT.begin();
//   WIRE_PORT.setClock(400000);
// #endif

// #ifndef QUAT_ANIMATION
//   //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
// #endif

//   bool initialized = false;
//   while (!initialized)
//   {

//     // Initialize the ICM-20948
//     // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
// #ifdef USE_SPI
//     myICM.begin(CS_PIN, SPI_PORT);
// #else
//     myICM.begin(WIRE_PORT, AD0_VAL);
// #endif

// #ifndef QUAT_ANIMATION
//     SERIAL_PORT.print(F("Initialization of the sensor returned: "));
//     SERIAL_PORT.println(myICM.statusString());
// #endif
//     if (myICM.status != ICM_20948_Stat_Ok)
//     {
// #ifndef QUAT_ANIMATION
//       SERIAL_PORT.println(F("Trying again..."));
// #endif
//       delay(500);
//     }
//     else
//     {
//       initialized = true;
//     }
//   }

// #ifndef QUAT_ANIMATION
//   SERIAL_PORT.println(F("Device connected!"));
// #endif

//   bool success = true; // Use success to show if the DMP configuration was successful

//   // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
//   success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

//   // DMP sensor options are defined in ICM_20948_DMP.h
//   //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
//   //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
//   //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
//   //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
//   //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
//   //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
//   //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
//   //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
//   //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
//   //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
//   //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
//   //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
//   //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
//   //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
//   //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

//   // Enable the DMP Game Rotation Vector sensor
//   success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

//   // Enable any additional sensors / features
//   //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
//   //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
//   //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

//   // Configuring DMP to output data at multiple ODRs:
//   // DMP is capable of outputting multiple sensor data at different rates to FIFO.
//   // Setting value can be calculated as follows:
//   // Value = (DMP running rate / ODR ) - 1
//   // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
//   success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
//   //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
//   //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
//   //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
//   //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
//   //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

//   // Enable the FIFO
//   success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

//   // Enable the DMP
//   success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

//   // Reset DMP
//   success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

//   // Reset FIFO
//   success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

//   // Check success
//   if (success)
//   {
// #ifndef QUAT_ANIMATION
//     SERIAL_PORT.println(F("DMP enabled!"));
// #endif
//   }
//   else
//   {
//     SERIAL_PORT.println(F("Enable DMP failed!"));
//     SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
//     while (1)
//       ; // Do nothing more
//   }


Serial.begin(115200);
ledcSetup(l_actuator_pwm_channel,frequency_actuator,resolution_actuator);  //channel 4
//ledcSetup(p_window_pwm_channel,frequency_actuator,resolution_actuator);    //channel 5
pinMode(L_ACTUATOR_DIR_PIN,OUTPUT);
ledcAttachPin(L_ACTUATOR_PWM_PIN,l_actuator_pwm_channel);

if(ps5.begin("00:BE:3B:F7:2D:51")){
    Serial.println("PS5 is connected.");

}
  //ps5.begin("D8:9E:61:A0:93:74"); //replace with your MAC address

ICM20948_SETUP_QUAT6(); 
 while(Initial_Pitch==0 && Initial_Yaw ==0)
  {
    current_millis = millis();
    while(millis()-current_millis<1500)
    {
      ICM20948_GET_READING_QUAT6(&Pitch, &Yaw);
    }
    ICM20948_GET_READING_QUAT6(&Pitch, &Yaw);
    if(fabs(Old_Pitch-Pitch)<0.05 && fabs(Old_Yaw-Yaw))
    {
      Initial_Pitch=Pitch;
      Initial_Yaw = Yaw;
      digitalWrite(2,LOW);
    }
    Old_Pitch=Pitch;
    Old_Yaw = Yaw;
  }
Serial.print("Initial Pitch: ");
Serial.println(Initial_Pitch);

Serial.print("Initial Yaw");
Serial.println(Initial_Yaw);

// while(Initial_Yaw==0)
//   {
//     current_millis = millis();
//     while(millis()-current_millis<1500)
//     {
//       ICM20948_GET_READING_QUAT6(&Yaw);
//     }
//     ICM20948_GET_READING_QUAT6(&Yaw);
//     if(fabs(Old_Yaw-Yaw)<0.05)
//     {
//       Initial_Yaw=Yaw;
//       digitalWrite(2,LOW);
//     }
//     Old_Yaw=Yaw;
//   }
// Serial.print("Initial Yaw: ");
// Serial.println(Initial_Yaw);



}

void loop()
{

while (ps5.isConnected() == true){
  ICM20948_GET_READING_QUAT6(&Pitch, &Yaw);
        Serial.print("Pitch_reading:\t");
        Serial.print(Pitch);
        Serial.print("\xC2\xB0"); //Print degree symbol
        Serial.println();
        Pitch_difference = Initial_Pitch - Pitch ;
        // Serial.print("Pitch diff: ");
        // Serial.print(Pitch_difference);
        // Serial.print("\xC2\xB0"); //Print degree symbol
        // Serial.println();

        Serial.print("Yaw_reading:\t");
        Serial.print(Yaw);
        Serial.print("\xC2\xB0"); //Print degree symbol
        Serial.println();
        Yaw_difference = Initial_Yaw - Yaw;
        // Serial.print("Yaw diff: ");
        // Serial.print(Yaw_difference);
        // Serial.print("\xC2\xB0"); //Print degree symbol
        // Serial.println();

        // Serial.print("Pitch_reading:\t");
        // Serial.print(Pitch);
        // Serial.print("\xC2\xB0"); //Print degree symbol
        // Serial.println();
        //Pitch_difference = Initial_Pitch - Pitch ;


if (ps5.PSButton())
    {
     ICM20948_GET_READING_QUAT6(&Pitch, &Yaw);
        
    }

if(ps5.Up())
    {
      digitalWrite(L_ACTUATOR_DIR_PIN,HIGH);
      ledcWrite(l_actuator_pwm_channel,80);
    }
    else if(ps5.Down())
    {
     
      digitalWrite(L_ACTUATOR_DIR_PIN,LOW);
      ledcWrite(l_actuator_pwm_channel,80);
       
    }
    else
    {
      ledcWrite(l_actuator_pwm_channel,0);
    }


if (ps5.Circle()) {
  digitalWrite(L_ACTUATOR_DIR_PIN, LOW);
  ledcWrite(l_actuator_pwm_channel, 80);
  //circlePressed = true; // if not pressed and ps5.circle is pressed

  

  // while (!(Pitch_difference > 45 && Pitch_difference < 45.9)) {
  //    ICM20948_GET_READING_QUAT6(&Pitch);
  //       Serial.print("Pitch_reading:\t");
  //       Serial.print(Pitch);
  //       Serial.print("\xC2\xB0"); //Print degree symbol
  //       Serial.println();
  //       Pitch_difference = Initial_Pitch - Pitch ;
  //       Serial.print("Pitch diff: ");
  //       Serial.print(Pitch_difference);
  //               Serial.print("\xC2\xB0"); //Print degree symbol
  //       Serial.println();
  // }
  //   ledcWrite(l_actuator_pwm_channel,0);

}



if(ps5.Triangle()){
while(Pitch !=0.98 || Pitch != -0.98 || Pitch != 0 ){
      digitalWrite(L_ACTUATOR_DIR_PIN,LOW);
      ledcWrite(l_actuator_pwm_channel,80);
    }

  }

}



}


    

