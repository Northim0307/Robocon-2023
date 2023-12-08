#include <ps5Controller.h>
#include "Arduino.h"
#include <EEPROM.h>
#include <ICM_20948.h>
#include <Robocon2023.h>
#include <ESP32Servo.h>
#include <VescUart.h>
#include <LiquidCrystal_I2C.h>

#define ULTRASONIC_LEFT_TRIG 4
#define ULTRASONIC_LEFT_ECHO 34
#define ULTRASONIC_RIGHT_TRIG 15
#define ULTRASONIC_RIGHT_ECHO 36

#define GRIPPING_DISTANCE 5
#define LEFT_WALL_DISTANCE 10
#define RIGHT_WALL_DISTANCE 10

#define SERVO_1_PIN  12
#define SERVO_2_PIN  14
#define SERVO_3_PIN  2
// #define LIMIT_SW_PIN 4

#define L_ACTUATOR_PWM_PIN 27
#define L_ACTUATOR_DIR_PIN 26

#define P_WINDOW_PWM_PIN 33
#define P_WINDOW_DIR_PIN 25

#define MOTOR1_PIN 5
#define MOTOR2_PIN 18
#define MOTOR3_PIN 19
#define MOTOR4_PIN 23

#define MOTOR1_STARTING 189
#define MOTOR2_STARTING 190
#define MOTOR3_STARTING 185
#define MOTOR4_STARTING 184

//clockwise need -4
//counter clockwise need +3


#define RIGHT_KEYENCE_1_PIN 35
#define RIGHT_KEYENCE_2_PIN 34

#define SPEED_CHG_PER_YAW 1

#define DEFAULT_FLIPSKY_RPM 10500
#define FLIPSKY_HIGHEST_RPM 24500
#define FLIPSKY_LOWEST_RPM 3500

// #define Pole_Distance 100 //in cm

#define MAX_YAW_TOLERANCE 1
#define MAX_PITCH_TOLERANCE 0.5
#define MAX_LIDAR_TOLERANCE 30





VescUart vesc1;

char Debug_Input;
bool Robot_Starting = true;
// bool Upperlvl_Robot_Starting = true;
// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

int motor1_pwm_channel     = 3;
int motor2_pwm_channel     = 4;
int motor3_pwm_channel     = 5;
int motor4_pwm_channel     = 6;
int l_actuator_pwm_channel = 7;
int p_window_pwm_channel   = 10;

int frequency_actuator     = 500;
int resolution_actuator = 8;

int frequency_dji = 500;
int resolution_dji = 8; //max is 255
// int resolution_dji = 16; //max is 65535


double Initial_Yaw=0;
double Old_Yaw, Yaw =0, Yaw_difference =0;
double Initial_Pitch=0;
double Old_Pitch, Pitch = 0, Pitch_difference = 0;
double Lidar_Aiming_Yaw;
double Extra_Yaw_Angle =0;


unsigned long current_millis;

bool Grip_Close=true;



int Robot_dir;
int Motor_dir[4];

float Magnitude_L, Angle_L;
float Magnitude_R, Angle_R;


int pulse_M[4];
int default_M[4]= {MOTOR1_STARTING,MOTOR2_STARTING,MOTOR3_STARTING,MOTOR4_STARTING};

char input;

int speed_count = 0;

float rpm = 0;

int DJI_Increase_Speed=0;
double Target_Yaw=0;
double Extra_Target_Yaw =0;
double Target_Pitch = 0;
int Pole_Distance = 0;

bool Fkipsky_Chg_Spd=false;
bool Left_Pressed = false;
bool Right_Pressed = false;
bool L1_Pressed = false;
bool L2_Pressed = false;
bool R1_Pressed = false;
bool R2_Pressed = false;
bool Circle_Pressed = false;
bool Touchpad_Pressed = false;
bool Square_Pressed = false;
bool Options_Pressed = false;
bool Share_Pressed = false;
bool Confirm_Lidar = false;

bool Grip_Servo = true; //true is close state

bool Auto_Aim;
bool Lidar_Aim_Adjustment = false;
bool Pitch_Adjustment = false;
bool Yaw_Adjustment = false;
bool Extra_Yaw_Adjustment = false;
bool Switching_Mode = false;
bool Aiming_Mode = false;
bool Right_Keyence_1 =false;
bool Right_Keyence_2 =false;
bool Position_Reached = false;
bool Aiming_Pole_4;
bool Aiming_Pole_5;
bool Aiming_Pole_6;
bool Aiming_Pole_7;
bool Aiming_Pole_8;

bool Waiting;


char Lidar_signal;

float Distance_Left;
float Distance_Right;

int Lidar_Distance;

struct Pole_Parameters{
  double Pitch_Pole;
  double Yaw_Pole;
  float Flipsky_RPM_Pole;
  double Extra_Yaw_Pole;
  int Distance_Pole;
};

Pole_Parameters Pole_4 = { //go right 3 timees
  36,       //Pitch
  173,       //Yaw
  8800,    //Flipsky RPM //
  6,        //Extra Yaw
  110        //Pole Distance
};

Pole_Parameters Pole_5 = { //go right 6 times
  36,       //Pitch            //height 24.5
  -173,       //Yaw
  8900,    //Flipsky RPM        23 time faster
  15,        //Extra Yaw
  110        //Pole Distance
};

Pole_Parameters Pole_6 = {
  51,       //Pitch
  -70,       //Yaw
  16900,    //Flipsky RPM
  0,        //Extra Yaw
  170       //Pole Distance
};

Pole_Parameters Pole_7 = {//turn right  3 times
  36,       //Pitch
  7,       //Yaw
  10300,    //Flipsky RPM
  6,        //Extra Yaw
  160        //Pole Distance
};

Pole_Parameters Pole_8 = { //go right 7 times
  39,       //Pitch
  -7,       //Yaw
  11000,    //Flipsky RPM
  15,        //Extra Yaw
  160       //Pole Distance
};


//using myservo will automatically search for avaiable pwm channel start from channel 0
Servo myservo1;
Servo myservo2;
Servo myservo3;


void setup()
{
  Robot_Starting = true;
  EEPROM.begin(250);

  lcd.init();
  // turn on LCD backlight                      
  lcd.backlight();

  lcd.setCursor(0, 0);
  // print message
  lcd.print("LCD INITIALISED");
  
  ledcSetup(motor1_pwm_channel,frequency_dji,resolution_dji);                //channel 0
  ledcSetup(motor2_pwm_channel,frequency_dji,resolution_dji);                //channel 1
  ledcSetup(motor3_pwm_channel,frequency_dji,resolution_dji);                //channel 2
  ledcSetup(motor4_pwm_channel,frequency_dji,resolution_dji);                //channel 3
  ledcSetup(l_actuator_pwm_channel,frequency_actuator,resolution_actuator);  //channel 4
  ledcSetup(p_window_pwm_channel,frequency_actuator,resolution_actuator);    //channel 5
  Serial.begin(115200);
  Serial2.begin(115200); //serial1

  vesc1.setSerialPort(&Serial2); //&serial

  // pinMode(LIMIT_SW_PIN, INPUT_PULLUP);
  pinMode(L_ACTUATOR_DIR_PIN,OUTPUT);
  pinMode(P_WINDOW_DIR_PIN,OUTPUT);

  pinMode(ULTRASONIC_LEFT_TRIG,OUTPUT);
  pinMode(ULTRASONIC_LEFT_ECHO,INPUT);
  // ps5.begin("00:BE:3B:F7:2D:51"); //WHITE CONTROLLER
  ps5.begin("1C:15:1F:41:39:0A"); //BLACK CONTROLLER
  //ps5.begin("D8:9E:61:A0:93:74"); //replace with your MAC address

  ICM20948_SETUP_QUAT6();

  // pinMode(2,OUTPUT);
  // digitalWrite(2,HIGH);


  while(Initial_Yaw==0 || Initial_Pitch==0 )
  {
    current_millis = millis();
    while(millis()-current_millis<1500)
    {
      ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
    }
    ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);

    if(ps5.Cross())
    {
      Initial_Yaw=Yaw;
      Initial_Pitch=Pitch;
      break;
    }
    if(fabs(Old_Yaw-Yaw)<0.05 && fabs(Old_Pitch-Pitch)<0.05)
    {
      Initial_Yaw=Yaw;
      Initial_Pitch=Pitch;
    }
    Old_Yaw = Yaw;
    Old_Pitch = Pitch;
  }
  

  myservo1.attach(SERVO_1_PIN); // attach the servo signal pin to GPIO 12
  myservo2.attach(SERVO_2_PIN); // attach the servo signal pin to GPIO 14
  myservo3.attach(SERVO_3_PIN); // attach the servo signal pin to GPIO 4

  ledcAttachPin(MOTOR1_PIN,motor1_pwm_channel);
  ledcAttachPin(MOTOR2_PIN,motor2_pwm_channel);
  ledcAttachPin(MOTOR3_PIN,motor3_pwm_channel);
  ledcAttachPin(MOTOR4_PIN,motor4_pwm_channel);
  ledcAttachPin(L_ACTUATOR_PWM_PIN,l_actuator_pwm_channel);
  ledcAttachPin(P_WINDOW_PWM_PIN,p_window_pwm_channel);
  
  // attachInterrupt(digitalPinToInterrupt(LIMIT_SW_PIN),Stop_P_Window,FALLING);

  //Serial.print("Initial Yaw: ");
  //Serial.println(Initial_Yaw);

  pulse_M[0]=default_M[0];
  pulse_M[1]=default_M[1];
  pulse_M[2]=default_M[2];
  pulse_M[3]=default_M[3];

  // pulse = 48639;

  ledcWrite(motor1_pwm_channel,pulse_M[0]=MOTOR1_STARTING);
  ledcWrite(motor2_pwm_channel,pulse_M[1]=MOTOR2_STARTING);
  ledcWrite(motor3_pwm_channel,pulse_M[2]=MOTOR3_STARTING);
  ledcWrite(motor4_pwm_channel,pulse_M[3]=MOTOR4_STARTING);

  lcd.setCursor(0, 0);   //<<<<<<<<
  lcd.print("Lid:");          //<<<<<<<<
  lcd.setCursor(8, 0);
  lcd.print("Yaw:"); 
  lcd.setCursor(0, 1);   //<<<<<<<<
  lcd.print("Pit:");          //<<<<<<<<
  lcd.setCursor(8, 1);
  lcd.print("RPM:"); 


}

// void Stop_P_Window()
// {
//   if(digitalRead(LIMIT_SW_PIN)==0)
//   {
//     ledcWrite(p_window_pwm_channel,0);
//   }
// }

void loop()
{
  

  while (ps5.isConnected() == true) 
  {
      ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
      Lidar_Distance = Get_Lidar_data();

      Pitch_difference = Initial_Pitch - Pitch ;
      Yaw_difference = Initial_Yaw - Yaw ;
      if(Yaw_difference>=180)
      {
        Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
      }
      else if (Yaw_difference<-180)
      {
        Yaw_difference = Yaw_difference + 360;
      }

      lcd.setCursor(0, 0);
          // print message
      lcd.print("Lid:");
      lcd.print("    ");
      lcd.setCursor(4, 0);
      lcd.print(Lidar_Distance);

      if(Aiming_Mode == false)
      {
        lcd.setCursor(8, 0);
        lcd.print("Yaw:");
        lcd.print("    ");
        lcd.setCursor(12, 0);
        lcd.print(Yaw_difference,1);
      }
      else
      {
        Extra_Yaw_Angle = Yaw_difference - Lidar_Aiming_Yaw;
        lcd.setCursor(8, 0);
        lcd.print("ExY:");
        lcd.print("    ");
        lcd.setCursor(12, 0);
        lcd.print(Extra_Yaw_Angle,1);
      }


      lcd.setCursor(0, 1);   //<<<<<<<<
      lcd.print("Pit:"); 
      lcd.print("    ");
      lcd.setCursor(4, 1);
      lcd.print(Pitch_difference,1); 
      
      lcd.setCursor(8, 1);
      lcd.print("RPM"); 
      lcd.print(rpm); 

  if(ps5.Touchpad())
    {
      if(Touchpad_Pressed==false)
      {
        Touchpad_Pressed = true;
        Aiming_Mode = !Aiming_Mode;
        Switching_Mode = true;
      }
    }
    else
    {
      Touchpad_Pressed = false;
    }
    if(Aiming_Mode==false)
    {
      if(Switching_Mode == true)
      {

        rpm = 0;
        Switching_Mode = false;
      }
      vesc1.setRPM(rpm);			  
      ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
      // Lidar_Distance = Get_Lidar_data();
      //Serial.print("Yaw_reading:\t");
      //Serial.print(Yaw);
      //Serial.print("\xC2\xB0"); //Print degree symbol
      //Serial.println();
      Yaw_difference = Initial_Yaw - Yaw ;
      if(Yaw_difference>=180)
      {
        Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
      }
      else if (Yaw_difference<-180)
      {
        Yaw_difference = Yaw_difference + 360;
      }

      


      

      // Serial.print("Yaw:\t");
      // Serial.print(Yaw_difference);
      // Serial.print("\xC2\xB0"); //Print degree symbol
      // Serial.println();

      if (ps5.PSButton())
      {
        ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
        Initial_Yaw = Yaw;
        //Serial.print("Initial Yaw: ");
        //Serial.println(Initial_Yaw);
      }

      if (ps5.Square())
      {
        rpm = DEFAULT_FLIPSKY_RPM;
        vesc1.setRPM(rpm);
      } 

      if (ps5.Triangle())   //push servo
      {
        myservo1.write(13); // rotate the servo to 180 degrees to close
      myservo2.write(120);
      delay(300);
      myservo1.write(5); // rotate the servo to 180 degrees to close
      myservo2.write(128); // rotate the servo to 180 degrees to close
        current_millis = millis();
        while(millis()-current_millis<500)
        {
          vesc1.setRPM(rpm);
        }
        myservo3.write(25);
        current_millis = millis();
        while(millis()-current_millis<550)
        {
          vesc1.setRPM(rpm);
        }
        myservo3.write(165);
      }
      
      if (ps5.Circle() )
      {
        if(Circle_Pressed == false)
        {
          Circle_Pressed = true;
          if(Grip_Servo ==false)
          {
            // while(ps5.Circle())
            // {
            //   Distance_Left = Read_Ultrasonic(ULTRASONIC_LEFT_TRIG,ULTRASONIC_LEFT_ECHO);
            //   Serial.print("Distance Left:");
            //   Serial.println(Distance_Left);

            //   if(fabs(Distance_Left-GRIPPING_DISTANCE)>1)
            //   {
            //     //go to left
            //     if(Distance_Left>4)
            //     {
            //       DJI_Increase_Speed = -7;
            //       Magnitude_L=90;
            //       Angle_L = 180;
            //       calc_robot_dir ( &Magnitude_L , &Angle_L, pulse_M , default_M,Motor_dir,DJI_Increase_Speed);

            //       ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
            //       Yaw_difference = Initial_Yaw - Yaw ;
            //       if(Yaw_difference>=180)
            //       {
            //         Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
            //       }
            //       else if (Yaw_difference<-180)
            //       {
            //         Yaw_difference = Yaw_difference + 360;
            //       }

            //       for(i=0;i<4;i++)
            //       {
            //         Setpoint_Adjust_Yaw (Yaw_difference, i,Motor_dir[i], pulse_M , SPEED_CHG_PER_YAW );
            //       }

            //       for(i = 0; i < 4; i++)
            //       {
            //         ledcWrite(i+3,pulse_M[i]);
            //       }
            //       Serial.println("Going to Left");
            //     }
            //     else
            //     //go to right
            //     {
            //        DJI_Increase_Speed = -7;
            //       Magnitude_L=90;
            //       Angle_L = 0;
            //       calc_robot_dir ( &Magnitude_L , &Angle_L, pulse_M , default_M,Motor_dir,DJI_Increase_Speed);

            //       ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
            //       Yaw_difference = Initial_Yaw - Yaw ;
            //       if(Yaw_difference>=180)
            //       {
            //         Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
            //       }
            //       else if (Yaw_difference<-180)
            //       {
            //         Yaw_difference = Yaw_difference + 360;
            //       }

            //       for(i=0;i<4;i++)
            //       {
            //         Setpoint_Adjust_Yaw (Yaw_difference, i,Motor_dir[i], pulse_M , SPEED_CHG_PER_YAW );
            //       }

            //       for(i = 0; i < 4; i++)
            //       {
            //         ledcWrite(i+3,pulse_M[i]);
            //       }
            //     }
            //   }
            //   else
            //   {
            //     Serial.println("Target Reached");
            //     for(i = 0; i < 4; i++)
            //     {
            //       pulse_M[i]=default_M[i];
            //       ledcWrite(i+3,pulse_M[i]);
            //     }
            //     DJI_Increase_Speed = 0;
            //     break;
            //   }
            // }
            Grip_Servo = true;
            myservo1.write(5); // rotate the servo to 180 degrees to close
          myservo2.write(128); // rotate the servo to 180 degrees to close
          }
          else
          {
            Grip_Servo = false;
            myservo1.write(32); // rotate the servo to 0 degrees to open
            myservo2.write(102);
          }
        }
      }
      else
      {
        Circle_Pressed = false;
      }

      if (ps5.R1())
      {
        // if(Robot_Starting == true)
        // {
        //   Target_Pitch = 10;
        //   ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
        //   Pitch_difference = Initial_Pitch - Pitch;

        //   if (fabs(Target_Pitch - Pitch_difference ) > MAX_PITCH_TOLERANCE ) 
        //   {
        //     if(ps5.Cross())
        //     {
        //       ledcWrite(l_actuator_pwm_channel,0);
        //       Robot_Starting = false;
        //     }
        //     if(Pitch_difference<Target_Pitch)
        //     {
        //       digitalWrite(L_ACTUATOR_DIR_PIN,HIGH);
        //     }
        //     else
        //     {
        //       digitalWrite(L_ACTUATOR_DIR_PIN,LOW);
        //     }
        //     ledcWrite(l_actuator_pwm_channel, 140);
        //   }
        //   else
        //   {
        //     ledcWrite(l_actuator_pwm_channel,0);
        //     Robot_Starting = false;
        //   }
        // }else
        // {
          digitalWrite(P_WINDOW_DIR_PIN,HIGH);
          ledcWrite(p_window_pwm_channel,120);
        // }
      }
      else if(ps5.R2())
      {
          // digitalWrite(P_WINDOW_DIR_PIN,LOW);
          // ledcWrite(p_window_pwm_channel,120);
          current_millis = millis();
          while(millis() - current_millis <= 785)
          {
            if(ps5.Cross())
            {
              ledcWrite(p_window_pwm_channel,0); //CW
              break;
            }
            digitalWrite(P_WINDOW_DIR_PIN,LOW);
            ledcWrite(p_window_pwm_channel,120); //CW
          }
      }
      else
      {
        ledcWrite(p_window_pwm_channel,0);
      }

      if(ps5.Up())
      {
        digitalWrite(L_ACTUATOR_DIR_PIN,HIGH);
        ledcWrite(l_actuator_pwm_channel,140);
      }
      else if(ps5.Down())
      {
        digitalWrite(L_ACTUATOR_DIR_PIN,LOW);
        ledcWrite(l_actuator_pwm_channel,140);
      }
      else
      {
        // if(Robot_Starting == false)
        ledcWrite(l_actuator_pwm_channel,0);
      }

      if(ps5.Left())
      {
        Target_Pitch = 3;
        Pitch_Adjustment = true;
      }

      if(ps5.Right())
      {
        Target_Pitch = 15;
        Pitch_Adjustment = true;
      }


      // if (ps5.Left())
      // {
      //   Distance_Left = Read_Ultrasonic(ULTRASONIC_LEFT_TRIG,ULTRASONIC_LEFT_ECHO);
      //   Position_Reached = false;
      //   Serial.print("Distance Left:");
      //   Serial.println(Distance_Left);

      //   while(Position_Reached == false)
      //   {

      //     if(fabs(Distance_Left-LEFT_WALL_DISTANCE)>1)
      //     {
      //       //go to left
      //       if(Distance_Left>LEFT_WALL_DISTANCE)
      //       {
      //         DJI_Increase_Speed = -7;
      //         Magnitude_L=90;
      //         Angle_L = 180;
      //         calc_robot_dir ( &Magnitude_L , &Angle_L, pulse_M , default_M,Motor_dir,DJI_Increase_Speed);

      //         ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
      //         Yaw_difference = Initial_Yaw - Yaw ;
      //         if(Yaw_difference>=180)
      //         {
      //           Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
      //         }
      //         else if (Yaw_difference<-180)
      //         {
      //           Yaw_difference = Yaw_difference + 360;
      //         }

      //         for(i=0;i<4;i++)
      //         {
      //           Setpoint_Adjust_Yaw (Yaw_difference, i,Motor_dir[i], pulse_M , SPEED_CHG_PER_YAW );
      //         }

      //         for(i = 0; i < 4; i++)
      //         {
      //           ledcWrite(i+3,pulse_M[i]);
      //         }
      //         Serial.println("Going to Left");
      //       }
      //       else
      //       //go to right
      //       {
      //           DJI_Increase_Speed = -7;
      //         Magnitude_L=90;
      //         Angle_L = 0;
      //         calc_robot_dir ( &Magnitude_L , &Angle_L, pulse_M , default_M,Motor_dir,DJI_Increase_Speed);

      //         ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
      //         Yaw_difference = Initial_Yaw - Yaw ;
      //         if(Yaw_difference>=180)
      //         {
      //           Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
      //         }
      //         else if (Yaw_difference<-180)
      //         {
      //           Yaw_difference = Yaw_difference + 360;
      //         }

      //         for(i=0;i<4;i++)
      //         {
      //           Setpoint_Adjust_Yaw (Yaw_difference, i,Motor_dir[i], pulse_M , SPEED_CHG_PER_YAW );
      //         }

      //         for(i = 0; i < 4; i++)
      //         {
      //           ledcWrite(i+3,pulse_M[i]);
      //         }
      //       }
      //     }
      //     else
      //     {
      //       Serial.println("Target Reached");
      //       for(i = 0; i < 4; i++)
      //       {
      //         pulse_M[i]=default_M[i];
      //         ledcWrite(i+3,pulse_M[i]);
      //       }
      //       Position_Reached = true;
      //       DJI_Increase_Speed = 0;
      //       break;
      //     }
      //   }
      // }

      // if (ps5.Right())
      // {
      //   Distance_Right = Read_Ultrasonic(ULTRASONIC_RIGHT_TRIG,ULTRASONIC_RIGHT_ECHO);
      //   Position_Reached = false;
      //   Serial.print("Distance Right:");
      //   Serial.println(Distance_Right);

      //   while(Position_Reached == false)
      //   {

      //     if(fabs(Distance_Right-RIGHT_WALL_DISTANCE)>1)
      //     {
      //       //go to left
      //       if(Distance_Right>RIGHT_WALL_DISTANCE)
      //       {
      //         DJI_Increase_Speed = -7;
      //         Magnitude_L=90;
      //         Angle_L = 0;
      //         calc_robot_dir ( &Magnitude_L , &Angle_L, pulse_M , default_M,Motor_dir,DJI_Increase_Speed);

      //         ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
      //         Yaw_difference = Initial_Yaw - Yaw ;
      //         if(Yaw_difference>=180)
      //         {
      //           Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
      //         }
      //         else if (Yaw_difference<-180)
      //         {
      //           Yaw_difference = Yaw_difference + 360;
      //         }

      //         for(i=0;i<4;i++)
      //         {
      //           Setpoint_Adjust_Yaw (Yaw_difference, i,Motor_dir[i], pulse_M , SPEED_CHG_PER_YAW );
      //         }

      //         for(i = 0; i < 4; i++)
      //         {
      //           ledcWrite(i+3,pulse_M[i]);
      //         }
      //         Serial.println("Going to Left");
      //       }
      //       else
      //       //go to right
      //       {
      //           DJI_Increase_Speed = -7;
      //         Magnitude_L=90;
      //         Angle_L = 180;
      //         calc_robot_dir ( &Magnitude_L , &Angle_L, pulse_M , default_M,Motor_dir,DJI_Increase_Speed);

      //         ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
      //         Yaw_difference = Initial_Yaw - Yaw ;
      //         if(Yaw_difference>=180)
      //         {
      //           Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
      //         }
      //         else if (Yaw_difference<-180)
      //         {
      //           Yaw_difference = Yaw_difference + 360;
      //         }

      //         for(i=0;i<4;i++)
      //         {
      //           Setpoint_Adjust_Yaw (Yaw_difference, i,Motor_dir[i], pulse_M , SPEED_CHG_PER_YAW );
      //         }

      //         for(i = 0; i < 4; i++)
      //         {
      //           ledcWrite(i+3,pulse_M[i]);
      //         }
      //       }
      //     }
      //     else
      //     {
      //       Serial.println("Target Reached");
      //       for(i = 0; i < 4; i++)
      //       {
      //         pulse_M[i]=default_M[i];
      //         ledcWrite(i+3,pulse_M[i]);
      //       }
      //       Position_Reached = true;
      //       DJI_Increase_Speed = 0;
      //       break;
      //     }
      //   }
      // }

      if (ps5.L2())
      {
            DJI_Increase_Speed=10;
      }
      else if(ps5.L1())
      {
        DJI_Increase_Speed= -5;
      }
      else
      {
        DJI_Increase_Speed= 0;
      }

      if(Pitch_Adjustment==true)
      {

        ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
        Pitch_difference = Initial_Pitch - Pitch;

        if (fabs(Target_Pitch - Pitch_difference ) > MAX_PITCH_TOLERANCE ) 
        {
          if(ps5.Cross())
          {
            ledcWrite(l_actuator_pwm_channel,0);
            Pitch_Adjustment = false;
          }
          if(Pitch_difference<Target_Pitch)
          {
            digitalWrite(L_ACTUATOR_DIR_PIN,HIGH);
          }
          else
          {
            digitalWrite(L_ACTUATOR_DIR_PIN,LOW);
          }
          ledcWrite(l_actuator_pwm_channel, 160);
        }
        else
        {
          ledcWrite(l_actuator_pwm_channel,0);
          Pitch_Adjustment = false;
        }

      }
  
  

      if (ps5.Cross())
      {
        rpm = 0;
        vesc1.setRPM(rpm);

        for(i = 0; i < 4; i++)
        {
          pulse_M[i]=default_M[i];
          ledcWrite(i+3,pulse_M[i]);
        }
        ledcWrite(l_actuator_pwm_channel,0);
        ledcWrite(p_window_pwm_channel,0);
        
      }

      Analog_Stick_Calc(float(ps5.LStickX()),float(ps5.LStickY()), &Magnitude_L , &Angle_L );
      Analog_Stick_Calc(float(ps5.RStickX()),float(ps5.RStickY()), &Magnitude_R , &Angle_R );
      calc_robot_dir ( &Magnitude_L , &Angle_L, pulse_M , default_M,Motor_dir,DJI_Increase_Speed);
      // }

      if(Magnitude_R>40)
      {
        if((Angle_R>-30 && Angle_R <=0)||(Angle_R>0 && Angle_R <=30))
        {
          for(i = 0;i<4;i++)
          {
          pulse_M[i] = default_M[i]+7;
          ledcWrite(i+3,pulse_M[i]);
          }
        }
        if((Angle_R>150 && Angle_R <=180)||(Angle_R>-180 && Angle_R <=-150))
        {
          for(i = 0;i<4;i++)
          {
          pulse_M[i] = default_M[i]-8;
          ledcWrite(i+3,pulse_M[i]);
          }
        }
        if((Angle_R>60 && Angle_R <=120)&&Fkipsky_Chg_Spd==false)
        {
          Fkipsky_Chg_Spd = true;
          rpm = rpm + 200;
          if(rpm>FLIPSKY_HIGHEST_RPM)
          {
            rpm=FLIPSKY_HIGHEST_RPM;

          }
          //Serial.print("rpm_now: ");
          //Serial.println(rpm);

        }
        if((Angle_R<-60 && Angle_R >=-120)&&Fkipsky_Chg_Spd==false)
        {
          Fkipsky_Chg_Spd = true;
          rpm = rpm - 200;
          if(rpm<FLIPSKY_LOWEST_RPM)
          {
            rpm=FLIPSKY_LOWEST_RPM;
          }
          //Serial.print("rpm_now: ");
          //Serial.println(rpm);

        }
      }
      else
      {
        Fkipsky_Chg_Spd = false;
      }
    

      if( Magnitude_L > 30) 
      {
        for(i=0;i<4;i++)
        {
        Setpoint_Adjust_Yaw (Yaw_difference, i,Motor_dir[i], pulse_M , SPEED_CHG_PER_YAW );
        //Serial.print("Setpoint of Motor:");
        //Serial.print(i);
        //Serial.print("is: ");
        //Serial.println (pulse_M[i]);
        }
        for(i = 0; i < 4; i++)
      {
          ledcWrite(i+3,pulse_M[i]);
      }
      }
        
      if( Magnitude_L < 30 && Magnitude_R <30) 
      {
        for(i = 0; i < 4; i++)
        {
          pulse_M[i]=default_M[i];
          ledcWrite(i+3,pulse_M[i]);
        }
      }
    }

    if(Aiming_Mode == true)
    {

      //action to indicate switching between navigation mode to aiming mode
      if(Switching_Mode == true)
      {
        rpm = Pole_7.Flipsky_RPM_Pole;
        Switching_Mode = false;
        // lcd.clear();
      }
      
      vesc1.setRPM(rpm);			  
      ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
      Lidar_Distance = Get_Lidar_data();

      Yaw_difference = Initial_Yaw - Yaw ;
      if(Yaw_difference>=180)
      {
        Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
      }
      else if (Yaw_difference<-180)
      {
        Yaw_difference = Yaw_difference + 360;
      }

      

      //Serial.print("Yaw:\t");
      //Serial.print(Yaw_difference);
      //Serial.print("\xC2\xB0"); //Print degree symbol
      //Serial.println();

      if (ps5.PSButton())
      {
        ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
        Initial_Yaw = Yaw;
        //Serial.print("Initial Yaw: ");
        //Serial.println(Initial_Yaw);
      }

      if(ps5.Right())
      {
        if(Right_Pressed == false)
        {
          Right_Pressed = true;
          ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
          Yaw_difference = Initial_Yaw - Yaw ;
          if(Yaw_difference>=180)
          {
            Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
          }
          else if (Yaw_difference<-180)
          {
            Yaw_difference = Yaw_difference + 360;
          }
          Target_Yaw =Yaw_difference+ 2;
          Yaw_Adjustment = true;
        }
      }
      else
      {
        Right_Pressed = false;
      }

      if(ps5.Left())
      {
        if(Left_Pressed == false)
        {
          Left_Pressed = true;
          ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
          Yaw_difference = Initial_Yaw - Yaw ;
          if(Yaw_difference>=180)
          {
            Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
          }
          else if (Yaw_difference<-180)
          {
            Yaw_difference = Yaw_difference + 360;
          }
          Target_Yaw = Yaw_difference - 2;
          Yaw_Adjustment = true;
        }
      }
      else
      {
        Left_Pressed = false;
      }

      if(ps5.Up())
      {
        Target_Pitch += 2 ;
        ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
        Pitch_difference = Initial_Pitch - Pitch;

        while (fabs(Pitch_difference - Target_Pitch) > MAX_PITCH_TOLERANCE ) 
        {
          if(ps5.Cross())
          {
            ledcWrite(l_actuator_pwm_channel,0);
            break;
          }
          if(Pitch_difference<Target_Pitch)
          {
            digitalWrite(L_ACTUATOR_DIR_PIN,HIGH);
          }
          else
          {
            digitalWrite(L_ACTUATOR_DIR_PIN,LOW);
          }
          ledcWrite(l_actuator_pwm_channel, 80);
          ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
          Pitch_difference = Initial_Pitch - Pitch ;
        }
        ledcWrite(l_actuator_pwm_channel,0);
      }
      else if(ps5.Down())
      {
        ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
        Target_Pitch -= 2;                    //pitch value increase if the shooter lower down
        Pitch_difference = Initial_Pitch - Pitch ;

        while (fabs(Pitch_difference - Target_Pitch) > MAX_PITCH_TOLERANCE ) 
        {
          if(ps5.Cross())
          {
            ledcWrite(l_actuator_pwm_channel,0);
            break;
          }
          if(Pitch_difference<Target_Pitch)
          {
            digitalWrite(L_ACTUATOR_DIR_PIN,HIGH);
          }
          else
          {
            digitalWrite(L_ACTUATOR_DIR_PIN,LOW);
          }
          ledcWrite(l_actuator_pwm_channel, 80);
          ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
          Pitch_difference = Initial_Pitch - Pitch ;
        }
        ledcWrite(l_actuator_pwm_channel,0);
      }
      else
      {
        ledcWrite(l_actuator_pwm_channel,0);
      }

       if (ps5.Cross())
      {
        rpm = 0;
        vesc1.setRPM(rpm);

        for(i = 0; i < 4; i++)
        {
          pulse_M[i]=default_M[i];
          ledcWrite(i+3,pulse_M[i]);
        }
        ledcWrite(l_actuator_pwm_channel,0);
        ledcWrite(p_window_pwm_channel,0);
        
      }

      if (ps5.Triangle())   //push servo
      {
        myservo1.write(13); // rotate the servo to 180 degrees to close
      myservo2.write(120);
      delay(300);
      myservo1.write(5); // rotate the servo to 180 degrees to close
      myservo2.write(128); // rotate the servo to 180 degrees to close
        current_millis = millis();
        while(millis()-current_millis<500)
        {
          vesc1.setRPM(rpm);
        }
        myservo3.write(25);
        current_millis = millis();
        while(millis()-current_millis<550)
        {
          vesc1.setRPM(rpm);
        }
        myservo3.write(165);
      }

      if (ps5.Circle() )
      {
        if(Circle_Pressed == false)
        {
          Circle_Pressed = true;
          if(Grip_Servo ==false)
          {
            Grip_Servo = true;
            myservo1.write(5); // rotate the servo to 180 degrees to close
          myservo2.write(128); // rotate the servo to 180 degrees to close
          }
          else
          {
            Grip_Servo = false;
            myservo1.write(32); // rotate the servo to 0 degrees to open
            myservo2.write(102);
          }
        }
      }
      else
      {
        Circle_Pressed = false;
      }

      // if (ps5.Circle())
      // {
      //   if(Circle_Pressed == false)
      //   {
      //     Circle_Pressed = true;
      //     Auto_Aim = true;  
      //     Target_Yaw = Pole_6.Yaw_Pole;
      //     rpm = Pole_6.Flipsky_RPM_Pole;
      //     Target_Pitch = Pole_6.Pitch_Pole;
      //     Extra_Target_Yaw = Pole_6.Extra_Yaw_Pole;
      //     Pole_Distance = Pole_6.Distance_Pole;
      //   }     
      // }
      // else
      // {
      //   Circle_Pressed = false;
      // }

      // if(ps5.Square())
      // {
      //   if(Square_Pressed == false)
      //   {
      //     Position_Reached = false;
      //     Square_Pressed = true;
      //     while(Position_Reached==false)
      //     {
      //       if(ps5.Cross())
      //       {
      //         for(i = 0; i<4 ; i++)
      //         {
      //           pulse_M[i] = default_M[i];
      //           ledcWrite(i+3,pulse_M[i]);
      //         }
      //         break;
      //       }
      //       Right_Keyence_1 = digitalRead(RIGHT_KEYENCE_1_PIN);
      //       Right_Keyence_2 = digitalRead(RIGHT_KEYENCE_2_PIN);
      //       if((Right_Keyence_1==HIGH)&& (Right_Keyence_2 == LOW))           //Keyence 1 will detect further distance, Keyence 2 will detect Close distance
      //       {
      //         Position_Reached = true;
      //       }
      //       else if((Right_Keyence_1==LOW)&&(Right_Keyence_1==LOW))
      //       {
      //         pulse_M[0] = default_M[0]-7;
      //         pulse_M[1] = default_M[1]-7;
      //         pulse_M[2] = default_M[2]+7;
      //         pulse_M[3] = default_M[3]+7;
      //         for(i = 0; i<4 ; i++)
      //         {
      //           ledcWrite(i+3,pulse_M[i]);
      //         }
      //       }
      //       else if((Right_Keyence_1==HIGH)&&(Right_Keyence_1==HIGH))
      //       {
      //         pulse_M[0] = default_M[0]+7;
      //         pulse_M[1] = default_M[1]+7;
      //         pulse_M[2] = default_M[2]-7;
      //         pulse_M[3] = default_M[3]-7;
      //         for(i = 0; i<4 ; i++)
      //         {
      //           ledcWrite(i+3,pulse_M[i]);
      //         }
      //       }
            
      //     }
          
      //   }     
      // }
      // else
      // {
      //   Square_Pressed = false;
      // }

      if (ps5.L1())
      {
        if(L1_Pressed == false)
        {
          L1_Pressed = true;
          Auto_Aim = true;
          Target_Yaw = Pole_4.Yaw_Pole;
          rpm = Pole_4.Flipsky_RPM_Pole;
          Target_Pitch = Pole_4.Pitch_Pole;
          Extra_Target_Yaw = Pole_4.Extra_Yaw_Pole;
          Pole_Distance = Pole_4.Distance_Pole;
        }     
      }
      else
      {
        L1_Pressed = false;
      }

      if (ps5.L2())
      {
        if(L2_Pressed == false)
        {
          L2_Pressed = true;
          Auto_Aim = true;
          Target_Yaw = Pole_7.Yaw_Pole;
          rpm = Pole_7.Flipsky_RPM_Pole;
          Target_Pitch = Pole_7.Pitch_Pole;
          Extra_Target_Yaw = Pole_7.Extra_Yaw_Pole;
          Pole_Distance = Pole_7.Distance_Pole;
        }     
      }
      else
      {
        L2_Pressed = false;
      }

      if (ps5.R1())
      {
        if(R1_Pressed == false)
        {
          R1_Pressed = true;
          Auto_Aim = true;
          Target_Yaw = Pole_5.Yaw_Pole;
          rpm = Pole_5.Flipsky_RPM_Pole;
          Target_Pitch = Pole_5.Pitch_Pole;
          Extra_Target_Yaw = Pole_5.Extra_Yaw_Pole;
          Pole_Distance = Pole_5.Distance_Pole;
        }     
      }
      else
      {
        R1_Pressed = false;
      }

      if (ps5.R2())
      {
        if(R2_Pressed == false)
        {
          R2_Pressed = true;
          Auto_Aim = true;
          Target_Yaw = Pole_8.Yaw_Pole;
          rpm = Pole_8.Flipsky_RPM_Pole;
          Target_Pitch = Pole_8.Pitch_Pole;
          Extra_Target_Yaw = Pole_8.Extra_Yaw_Pole;
          Pole_Distance = Pole_8.Distance_Pole;
        }     
      }
      else
      {
        R2_Pressed = false;
      }

      Analog_Stick_Calc(float(ps5.LStickX()),float(ps5.LStickY()), &Magnitude_L , &Angle_L );
      Analog_Stick_Calc(float(ps5.RStickX()),float(ps5.RStickY()), &Magnitude_R , &Angle_R );
      calc_robot_dir ( &Magnitude_L , &Angle_L, pulse_M , default_M,Motor_dir,DJI_Increase_Speed);

      

      if(Auto_Aim == true)
      {
        Lidar_Aim_Adjustment = true;
        Yaw_Adjustment = true;
        Extra_Yaw_Adjustment = true;
        Pitch_Adjustment = true;


        ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
        Yaw_difference = Initial_Yaw - Yaw ;
        if(Yaw_difference>=180)
        {
          Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
        }
        else if (Yaw_difference<-180)
        {
          Yaw_difference = Yaw_difference + 360;
        }

        if(Target_Yaw>Yaw_difference)
        {
          Robot_dir = CLOCKWISE;
        }
        else
        {
          Robot_dir = COUNTER_CLOCKWISE;
        }

        

        while(Lidar_Aim_Adjustment == true)
        {
          if (ps5.Cross())
          {
            for(i = 0; i<4 ; i++)
            {
              pulse_M[i] = default_M[i];
              ledcWrite(i+3,pulse_M[i]);
            }
            Lidar_Aim_Adjustment = false;
            break;
          }
          if(ps5.Left())
          {
            Robot_dir=CLOCKWISE;
          }
          if(ps5.Right())
          {
            Robot_dir=COUNTER_CLOCKWISE; 
          }
          Lidar_Distance = Get_Lidar_data();      

          if(abs(Lidar_Distance - Pole_Distance)>MAX_LIDAR_TOLERANCE)
          {
            if(Robot_dir==CLOCKWISE)
            {
              for(i = 0; i<4 ; i++)
              {
                pulse_M[i] = default_M[i]+7; //motor to counter-clockwise direction to make robot rotate clockwise
                ledcWrite(i+3,pulse_M[i]);
              }
            }
            else
            {
              for(i = 0; i<4 ; i++)
              {
                pulse_M[i] = default_M[i]-8; //motor to clockwise direction to make robot rotate counter clockwise
                ledcWrite(i+3,pulse_M[i]);
              }
            }
          }
          else
          {
            for(i = 0; i < 4; i++)
            {
              ledcWrite(i+3,default_M[i]);
            }

            lcd.setCursor(0, 0);
            lcd.print("Lid:");
            lcd.print("    ");
              // print message
            lcd.setCursor(4, 0);
            lcd.print(Lidar_Distance);



            Confirm_Lidar = false;
            while(Confirm_Lidar == false)
            {
              Lidar_Distance = Get_Lidar_data();

              
 

              if (ps5.Cross())
              {
                for(i = 0; i<4 ; i++)
                {
                  pulse_M[i] = default_M[i];
                  ledcWrite(i+3,pulse_M[i]);
                }
                Lidar_Aim_Adjustment = false;
                Confirm_Lidar = true;
             }
      

              while(ps5.Left())
              {
                Robot_dir=COUNTER_CLOCKWISE;
                for(i = 0; i<4 ; i++)
                {
                  pulse_M[i] = default_M[i]-8; //motor to clockwise direction to make robot rotate counter clockwise
                  ledcWrite(i+3,pulse_M[i]);
                }
                Confirm_Lidar = true;
              }
              while(ps5.Right())
              {
                Robot_dir=CLOCKWISE;
                for(i = 0; i<4 ; i++)
                {
                  pulse_M[i] = default_M[i]+7; //motor to counter-clockwise direction to make robot rotate clockwise
                  ledcWrite(i+3,pulse_M[i]);
                }
                Confirm_Lidar = true;
              }
              if(ps5.Square())
              {
                ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
                Yaw_difference = Initial_Yaw - Yaw ;
                if(Yaw_difference>=180)
                {
                  Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
                }
                else if (Yaw_difference<-180)
                {
                  Yaw_difference = Yaw_difference + 360;
                }
                Lidar_Aiming_Yaw = Yaw_difference;
                Confirm_Lidar = true;
                Lidar_Aim_Adjustment = false;
              }
            }
          }
        }
            // for(i = 0; i < 4; i++)
            // {
            //   ledcWrite(i+3,default_M[i]);
            // }
            // ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
            // Yaw_difference = Initial_Yaw - Yaw ;
            // if(Yaw_difference>=180)
            // {
            //   Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
            // }
            // else if (Yaw_difference<-180)
            // {
            //   Yaw_difference = Yaw_difference + 360;
            // }
            // if(fabs(Target_Yaw - Yaw_difference)<3)
            // {
            //   Lidar_Aim_Adjustment = false;
            //   Yaw_Adjustment = false;
            //   break;
            // }
            // else
            // {
            //   while(Yaw_Adjustment==true)
            //   {
            //     if (ps5.Cross())
            //     {
            //       for(i = 0; i<4 ; i++)
            //       {
            //         pulse_M[i] = default_M[i];
            //         ledcWrite(i+3,pulse_M[i]);
            //       }
            //       break;
            //     }
            //     ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
            //     Yaw_difference = Initial_Yaw - Yaw ;
            //     if(Yaw_difference>=180)
            //     {
            //       Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
            //     }
            //     else if (Yaw_difference<-180)
            //     {
            //       Yaw_difference = Yaw_difference + 360;
            //     }
            //     //Robot rotating to clockwise direction
            //     if(Target_Yaw - Yaw_difference>3)
            //     {
            //       for(i = 0; i<4 ; i++)
            //       {
            //         pulse_M[i] = default_M[i]+7; //motor to counter-clockwise direction to make robot rotate clockwise
            //         ledcWrite(i+3,pulse_M[i]);
            //       }
            //     }
            //     //Robot rotating to counter-clockwise direction
            //     else if(Target_Yaw - Yaw_difference<3)
            //     {
            //       for(i = 0; i<4 ; i++)
            //       {
            //         pulse_M[i] = default_M[i]-7; //motor to clockwise direction to make robot rotate counter clockwise
            //       ledcWrite(i+3,pulse_M[i]);
            //       }
            //     }
            //     else
            //     {
            //       Yaw_Adjustment = false;
            //       for(i = 0; i<4 ; i++)
            //       {
            //         pulse_M[i] = default_M[i]; //motor to clockwise direction to make robot rotate counter clockwise
            //         ledcWrite(i+3,pulse_M[i]);
            //       }
            //     }
            //   }
            // }
        //   }
        // }
        // Yaw_Adjustment = false; //now does not need to auto correct
        
        ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
        Yaw_difference = Initial_Yaw - Yaw ;
        if(Yaw_difference>=180)
        {
          Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
        }
        else if (Yaw_difference<-180)
        {
          Yaw_difference = Yaw_difference + 360;
        }
        Target_Yaw = Yaw_difference + Extra_Target_Yaw;



        // while(Extra_Yaw_Adjustment == true)
        // {
        //   if (ps5.Cross())
        //   {
        //     for(i = 0; i<4 ; i++)
        //     {
        //       pulse_M[i] = default_M[i];
        //       ledcWrite(i+3,pulse_M[i]);
        //     }
        //     Lidar_Aim_Adjustment = false;
        //     Extra_Yaw_Adjustment = false;
        //     break;
        //   }
        //   ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
        //   Yaw_difference = Initial_Yaw - Yaw ;
        //   if(Yaw_difference>=180)
        //   {
        //     Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
        //   }
        //   else if (Yaw_difference<-180)
        //   {
        //     Yaw_difference = Yaw_difference + 360;
        //   }
          
        //   if(fabs(Target_Yaw - Yaw_difference)>MAX_YAW_TOLERANCE)
        //   {
        //     //Robot rotating to clockwise direction
        //     if(Target_Yaw - Yaw_difference>0)
        //     {
        //       for(i = 0; i<4 ; i++)
        //       {
        //         pulse_M[i] = default_M[i]+7; //motor to counter-clockwise direction to make robot rotate clockwise
        //         ledcWrite(i+3,pulse_M[i]);
        //       }
        //     }
        //       //Robot rotating to counter-clockwise direction
        //     else
        //     {
        //       for(i = 0; i<4 ; i++)
        //       {
        //         pulse_M[i] = default_M[i]-8; //motor to clockwise direction to make robot rotate counter clockwise
        //         ledcWrite(i+3,pulse_M[i]);
        //       }
        //     }
        //   }
        //   else
        //   {
        //     for(i = 0; i<4 ; i++)
        //     {
        //       pulse_M[i] = default_M[i];
        //       ledcWrite(i+3,pulse_M[i]);
        //     }
        //     Extra_Yaw_Adjustment = false;
        //   }

        // }

        while(Pitch_Adjustment == true)
        {
          ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
          Pitch_difference = Initial_Pitch - Pitch ;

          while (fabs(Pitch_difference - Target_Pitch) > MAX_PITCH_TOLERANCE ) 
          {
            
            if (ps5.Cross())
            {
              ledcWrite(l_actuator_pwm_channel, 0);
              Lidar_Aim_Adjustment = false;
              Yaw_Adjustment = false;
              Extra_Yaw_Adjustment = false;
              Pitch_Adjustment = false;
              
              break;
            }

            if(Pitch_difference<Target_Pitch)
            {
              digitalWrite(L_ACTUATOR_DIR_PIN,HIGH);
            }
            else
            {
              digitalWrite(L_ACTUATOR_DIR_PIN,LOW);
            }
            ledcWrite(l_actuator_pwm_channel, 140);
            ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
            Pitch_difference = Initial_Pitch - Pitch ;
          }
          ledcWrite(l_actuator_pwm_channel,0);
          Pitch_Adjustment = false;
        }

        vesc1.setRPM(rpm);
        Auto_Aim = false;

      }

      while(Yaw_Adjustment==true)
      {
        if (ps5.Cross())
        {
          Target_Yaw=0;
          Yaw_Adjustment = false;
          for(i = 0; i<4 ; i++)
          {
            pulse_M[i] = default_M[i];
            ledcWrite(i+3,pulse_M[i]);
          }
          break;
        }
        ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
        Yaw_difference = Initial_Yaw - Yaw ;
        if(Yaw_difference>=180)
        {
          Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
        }
        else if (Yaw_difference<-180)
        {
          Yaw_difference = Yaw_difference + 360;
        }
        if(fabs(Target_Yaw - Yaw_difference)>MAX_YAW_TOLERANCE)
        {
          //Robot rotating to clockwise direction
          if(Target_Yaw - Yaw_difference>0)
          {
            for(i = 0; i<4 ; i++)
            {
            pulse_M[i] = default_M[i]+5; //motor to counter-clockwise direction to make robot rotate clockwise
            ledcWrite(i+3,pulse_M[i]);
            }
          }
          //Robot rotating to counter-clockwise direction
          else
          {
            for(i = 0; i<4 ; i++)
            {
            pulse_M[i] = default_M[i]-6; //motor to clockwise direction to make robot rotate counter clockwise
            ledcWrite(i+3,pulse_M[i]);
            }
          }
        }
        else
        {
          for(i = 0; i<4 ; i++)
          {
          pulse_M[i] = default_M[i];
          ledcWrite(i+3,pulse_M[i]);
          }
          Yaw_Adjustment = false;
        }
      }

      if(Magnitude_R>40)
      {
        if((Angle_R>-30 && Angle_R <=0)||(Angle_R>0 && Angle_R <=30))
        {
          for(i = 0;i<4;i++)
          {
          pulse_M[i] = default_M[i]+7;
          ledcWrite(i+3,pulse_M[i]);
          }
        }
        if((Angle_R>150 && Angle_R <=180)||(Angle_R>-180 && Angle_R <=-150))
        {
          for(i = 0;i<4;i++)
          {
          pulse_M[i] = default_M[i]-8;
          ledcWrite(i+3,pulse_M[i]);
          }
        }
        if((Angle_R>60 && Angle_R <=120)&&Fkipsky_Chg_Spd==false)
        {
          Fkipsky_Chg_Spd = true;
          rpm = rpm + 200;
          if(rpm>FLIPSKY_HIGHEST_RPM)
          {
            rpm=FLIPSKY_HIGHEST_RPM;

          }
          //Serial.print("rpm_now: ");
          //Serial.println(rpm);

        }
        if((Angle_R<-60 && Angle_R >=-120)&&Fkipsky_Chg_Spd==false)
        {
          Fkipsky_Chg_Spd = true;
          rpm = rpm - 200;
          if(rpm<FLIPSKY_LOWEST_RPM)
          {
            rpm=FLIPSKY_LOWEST_RPM;
          }
          //Serial.print("rpm_now: ");
          //Serial.println(rpm);

        }
      }
      else
      {
        Fkipsky_Chg_Spd = false;
      }
    

        

      // while(Target_Yaw!=0 )
      // {
      //   if (ps5.Cross())
      //   {
      //     Target_Yaw=0;
      //     for(i = 0; i<4 ; i++)
      //     {
      //       pulse_M[i] = default_M[i];
      //       ledcWrite(i+3,pulse_M[i]);
      //     }
      //     break;
      //   }
      //   ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
      //     Yaw_difference = Initial_Yaw - Yaw ;
      //     if(Yaw_difference>=180)
      //     {
      //       Yaw_difference = Yaw_difference- 360;    // if positive then is towards clockwise, if negative then is towards anticlockwise
      //     }
      //     else if (Yaw_difference<-180)
      //     {
      //       Yaw_difference = Yaw_difference + 360;
      //     }
      //   if(fabs(Target_Yaw - Yaw_difference)>1)
      //   {
      //     if(Target_Yaw - Yaw_difference>0)
      //     {
      //       for(i = 0; i<4 ; i++)
      //       {
      //       pulse_M[i] = default_M[i]+7;
      //       ledcWrite(i+3,pulse_M[i]);
      //       }
      //     }
      //     else
      //     {
      //       for(i = 0; i<4 ; i++)
      //       {
      //       pulse_M[i] = default_M[i]-8;
      //       ledcWrite(i+3,pulse_M[i]);
      //       }
      //     }
      //   }
      //   else
      //   {
      //     for(i = 0; i<4 ; i++)
      //     {
      //     pulse_M[i] = default_M[i];
      //     ledcWrite(i+3,pulse_M[i]);
      //     }
      //     Target_Yaw = 0;
      //     ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
      //     Initial_Yaw = Yaw;
      //   }

      // }

        
      
      if( Magnitude_L > 30) 
      {
        for(i=0;i<4;i++)
        {
        Setpoint_Adjust_Yaw (Yaw_difference, i,Motor_dir[i], pulse_M , SPEED_CHG_PER_YAW );
        //Serial.print("Setpoint of Motor:");
        //Serial.print(i);
        //Serial.print("is: ");
        //Serial.println (pulse_M[i]);
        }
        for(i = 0; i < 4; i++)
      {
          ledcWrite(i+3,pulse_M[i]);
      }
      }
        
      if( Magnitude_L < 30 && Magnitude_R <30) 
      {
        for(i = 0; i < 4; i++)
        {
          pulse_M[i]=default_M[i];
          ledcWrite(i+3,pulse_M[i]);
        }
      }
    
    }
    if (ps5.Share()) 
    {
      if (Share_Pressed == false) 
      {
        // Serial.println("Share Pressed");
        Share_Pressed = true;
        Waiting = true;
        while (Waiting == true) 
        {
          // Serial.println("Wwaitng for input");
          if (ps5.L1()) {
            Serial.println("L1 pressed");
            Aiming_Pole_4 = true;
            Waiting = false;
          }
          if (ps5.L2()) {
            Aiming_Pole_5 = true;
            Waiting = false;
          }
          if (ps5.Circle()) {
            Aiming_Pole_6 = true;
            Waiting = false;
          }
          if (ps5.R1()) {
            Aiming_Pole_7 = true;
            Waiting = false;
          }
          if (ps5.R2()) {
            Aiming_Pole_8 = true;
            Waiting = false;
          }
        }
        if (Aiming_Pole_4 == true) {
          // Serial.println("Loading Pole 4");
          EEPROM.get(0, Pole_4);
          // Serial.println("Sucessful Loading Pole 4");
          Aiming_Pole_4 = false;
        }
        if (Aiming_Pole_5 == true) {
          EEPROM.get(50, Pole_5);
          Aiming_Pole_5 = false;
        }
        if (Aiming_Pole_6 == true) {
          EEPROM.get(100, Pole_6);
          Aiming_Pole_6 = false;
        }
        if (Aiming_Pole_7 == true) {
          EEPROM.get(150, Pole_7);
          Aiming_Pole_7 = false;
        }
        if (Aiming_Pole_8 == true) {
          EEPROM.get(200, Pole_8);
          Aiming_Pole_8 = false;
        }
      }
    } else {
      Share_Pressed = false;
    }

    if (ps5.Options()) {
      if (Options_Pressed == false) {
        Options_Pressed = true;
        Waiting = true;
        // Serial.println("Options Pressed");
        while (Waiting == true) {
          // Serial.println("Waiting for input");
          if (ps5.L1()) {
            Aiming_Pole_4 = true;
            Waiting = false;
            Serial.println("L1 Pressed");
          }
          if (ps5.L2()) {
            Aiming_Pole_5 = true;
            Waiting = false;
          }
          if (ps5.Circle()) {
            Aiming_Pole_6 = true;
            Waiting = false;
          }
          if (ps5.R1()) {
            Aiming_Pole_7 = true;
            Waiting = false;
          }
          if (ps5.R2()) {
            Aiming_Pole_8 = true;
            Waiting = false;
          }
        }
        if (Aiming_Pole_4 == true) {
          Serial.println("Storing Pole 4 0");
          Pole_4.Pitch_Pole = Pitch_difference;
          Pole_4.Flipsky_RPM_Pole = rpm;
          Serial.println("Storing Pole 4 1");
          EEPROM.put(0, Pole_4);
          Serial.println("Storing Pole 4 2");
          EEPROM.commit();
          Serial.println("Storing Pole 4 3");
          Aiming_Pole_4 = false;
        }
        if (Aiming_Pole_5 == true) {


          Pole_5.Flipsky_RPM_Pole = rpm;
          Pole_5.Pitch_Pole = Pitch_difference;
          EEPROM.put(50, Pole_5);
          EEPROM.commit();
          Aiming_Pole_5 = false;
        }
        if (Aiming_Pole_6 == true) {
          Pole_6.Flipsky_RPM_Pole = rpm;
          Pole_6.Pitch_Pole = Pitch_difference;
          EEPROM.put(100, Pole_6);
          EEPROM.commit();
          Aiming_Pole_6 = false;
        }
        if (Aiming_Pole_7 == true) {
          Pole_7.Flipsky_RPM_Pole = rpm;
          Pole_7.Pitch_Pole = Pitch_difference;
          EEPROM.put(150, Pole_7);
          EEPROM.commit();
          Aiming_Pole_7 = false;
        }
        if (Aiming_Pole_8 == true) {
          Pole_8.Flipsky_RPM_Pole = rpm;
          Pole_8.Pitch_Pole = Pitch_difference;
          EEPROM.put(200, Pole_8);
          EEPROM.commit();
          Aiming_Pole_8 = false;
        }
      }
    } else {
      Options_Pressed = false;
    }

  }
  if(Serial.available())
  {
    //    struct Pole_Parameters{
    //   double Pitch_Pole;
    //   double Yaw_Pole;
    //   float Flipsky_RPM_Pole;
    //   double Extra_Yaw_Pole;
    //   int Distance_Pole;
    // };
    Debug_Input = Serial.read();
    if(Debug_Input == 'c')
    {
      EEPROM.get(0,Pole_4);
      Serial.println("Pole 4");
      Serial.print("Pitch");
      Serial.println(Pole_4.Pitch_Pole);
      Serial.print("rpm");
      Serial.println(Pole_4.Flipsky_RPM_Pole);
    }
    if(Debug_Input == '5')
    {
      EEPROM.get(50,Pole_5);
      Serial.println("Pole 5");
      Serial.print("Pitch");
      Serial.println(Pole_5.Pitch_Pole);
      Serial.print("rpm");
      Serial.println(Pole_5.Flipsky_RPM_Pole);
    }
    if(Debug_Input == '6')
    {
      EEPROM.get(100,Pole_6);
      Serial.println("Pole 6");
      Serial.print("Pitch");
      Serial.println(Pole_6.Pitch_Pole);
      Serial.print("rpm");
      Serial.println(Pole_6.Flipsky_RPM_Pole);
    }
    if(Debug_Input == '7')
    {
      EEPROM.get(150,Pole_7);
      Serial.println("Pole 7");
      Serial.print("Pitch");
      Serial.println(Pole_7.Pitch_Pole);
      Serial.print("rpm");
      Serial.println(Pole_7.Flipsky_RPM_Pole);
    }
    if(Debug_Input == '8')
    {
      EEPROM.get(200,Pole_8);
      Serial.println("Pole 8");
      Serial.print("Pitch");
      Serial.println(Pole_8.Pitch_Pole);
      Serial.print("rpm");
      Serial.println(Pole_8.Flipsky_RPM_Pole);
    }
  }
}

