// #include <mcp_can.h>
// #include <SPI.h>
#include <PID_v1.h>
#include <ps5Controller.h>
#include "Arduino.h"
// #include <MPU9250.h>
#include <EEPROM.h>
#include <ICM_20948.h>
#include <Robocon2023.h>
#include <ESP32Servo.h>
#include <VescUart.h>

#define SERVO_1_PIN  12
#define SERVO_2_PIN  14
#define SERVO_3_PIN  4
#define LIMIT_SW_PIN 2

#define L_ACTUATOR_PWM_PIN 27
#define L_ACTUATOR_DIR_PIN 26

#define P_WINDOW_PWM_PIN 33
#define P_WINDOW_DIR_PIN 25

#define MOTOR1_PIN 5
#define MOTOR2_PIN 18
#define MOTOR3_PIN 19
#define MOTOR4_PIN 23

#define MOTOR1_STARTING 187
#define MOTOR2_STARTING 191
#define MOTOR3_STARTING 196
#define MOTOR4_STARTING 184


#define SPEED_CHG_PER_YAW 1

#define DEFAULT_FLIPSKY_RPM 10500
#define FLIPSKY_HIGHEST_RPM 24500
#define FLIPSKY_LOWEST_RPM 3500

#define POLE_DISTANCE 100 //in cm


VescUart vesc1;

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

// const int OPTIMUM_SPEED_M[]= {1000,1000,1000,1000};         //200 if test on table

double Initial_Yaw=0;
double Old_Yaw, Yaw =0, Yaw_difference =0;
double Initial_Pitch=0;
double Old_Pitch, Pitch = 0, Pitch_difference = 0;
double Target_Pitch = 0 ;
unsigned long current_millis;

bool Grip_Close=true;

////test on table
//const double OPTIMUM_CLOCKWISE_CURRENT_M[]= {200,200,200,200};         //200 if test on table
//const double OPTIMUM_COUNTER_CLOCKWISE_CURRENT_M[]= {200,200,200,200};

//test on floor
// const double OPTIMUM_CLOCKWISE_CURRENT_M[]= {1100,1100,1100,1100};         //200 if test on table
// const double OPTIMUM_COUNTER_CLOCKWISE_CURRENT_M[]= {1100,1100,1100,1100};

int Robot_dir_now;
int Robot_dir_prev=STOP;
bool Robot_dir_chged;
bool Starting_Motor[4]= {true,true,true,true};
double CAN_Output_M[4];
int Motor_dir[4];
int Prev_Motor_dir[4]={2,2,2,2};
int Sample_Size = 1000;
double Optimum_Current[5][4]={0};



              

/*CAN BUS variables */

char msgString[128];          //An array of char to store the msg string for printing 
uint16_t feedback_M[4];


/*PID variables */

// bool PID_completed = false;

//Define Variables and initial tuning parameters for PID
// double Setpoint_M[4], Input_M[4], Output_M[4]; 
// double Kp_M1=0.5, Ki_M1=1.0, Kd_M1=0.00;       //Kp,Ki,Kd for motor 1
// double Kp_M2=0.5, Ki_M2=1.0, Kd_M2=0.00;       //Kp,Ki,Kd for motor 2
// double Kp_M3=0.5, Ki_M3=1.0, Kd_M3=0.00;       //Kp,Ki,Kd for motor 1
// double Kp_M4=0.5, Ki_M4=1.0, Kd_M4=0.00;       //Kp,Ki,Kd for motor 2

//Specify the links and Initiate the PID
// PID PID_M1(&Input_M[0], &Output_M[0], &Setpoint_M[0], Kp_M1, Ki_M1, Kd_M1,0, DIRECT);
// PID PID_M2(&Input_M[1], &Output_M[1], &Setpoint_M[1], Kp_M2, Ki_M2, Kd_M2,0, DIRECT);
// PID PID_M3(&Input_M[2], &Output_M[2], &Setpoint_M[2], Kp_M3, Ki_M3, Kd_M3,0, DIRECT);
// PID PID_M4(&Input_M[3], &Output_M[3], &Setpoint_M[3], Kp_M4, Ki_M4, Kd_M4,0, DIRECT);


float Magnitude_L, Angle_L;
float Magnitude_R, Angle_R;



int pulse_M[4];
int default_M[4]= {MOTOR1_STARTING,MOTOR2_STARTING,MOTOR3_STARTING,MOTOR4_STARTING};
float Rotate_angle=0;

char input;

int speed_count = 0;

float rpm = 0;

int DJI_Increase_Speed=0;

bool Fkipsky_Chg_Spd=false;
bool L1_Pressed = false;
bool L2_Pressed = false;
bool Circle_Pressed = false;

bool Grip_Servo = false; 



bool Aim_successful = false;
char Lidar_signal;

int Lidar_Distance;

  
  //using myservo will automatically search for avaiable pwm channel start from channel 0
  Servo myservo1;
  Servo myservo2;
  Servo myservo3;


void setup()
{
  ledcSetup(motor1_pwm_channel,frequency_dji,resolution_dji);                //channel 0
  ledcSetup(motor2_pwm_channel,frequency_dji,resolution_dji);                //channel 1
  ledcSetup(motor3_pwm_channel,frequency_dji,resolution_dji);                //channel 2
  ledcSetup(motor4_pwm_channel,frequency_dji,resolution_dji);                //channel 3
  ledcSetup(l_actuator_pwm_channel,frequency_actuator,resolution_actuator);  //channel 4
  ledcSetup(p_window_pwm_channel,frequency_actuator,resolution_actuator);    //channel 5
  Serial.begin(115200);
  Serial2.begin(115200); //serial1

  vesc1.setSerialPort(&Serial2); //&serial

  pinMode(LIMIT_SW_PIN, INPUT_PULLUP);
  pinMode(L_ACTUATOR_DIR_PIN,OUTPUT);
  pinMode(P_WINDOW_DIR_PIN,OUTPUT);
  

  myservo1.attach(SERVO_1_PIN); // attach the servo signal pin to GPIO 12
  myservo2.attach(SERVO_2_PIN); // attach the servo signal pin to GPIO 14
  myservo3.attach(SERVO_3_PIN); // attach the servo signal pin to GPIO 4

  ledcAttachPin(MOTOR1_PIN,motor1_pwm_channel);
  ledcAttachPin(MOTOR2_PIN,motor2_pwm_channel);
  ledcAttachPin(MOTOR3_PIN,motor3_pwm_channel);
  ledcAttachPin(MOTOR4_PIN,motor4_pwm_channel);
  ledcAttachPin(L_ACTUATOR_PWM_PIN,l_actuator_pwm_channel);
  ledcAttachPin(P_WINDOW_PWM_PIN,p_window_pwm_channel);
  
  attachInterrupt(digitalPinToInterrupt(LIMIT_SW_PIN),Stop_P_Window,FALLING);

  ps5.begin("00:BE:3B:F7:2D:51"); //WHITE CONTROLLER
  // ps5.begin("1C:15:1F:41:39:0A"); //BLACK CONTROLLER
  //ps5.begin("D8:9E:61:A0:93:74"); //replace with your MAC address
  //Serial.println("Ready.");

  // pinMode(2,OUTPUT);

  ICM20948_SETUP_QUAT6();

  // digitalWrite(2,HIGH);

  while(Initial_Yaw==0 || Initial_Pitch==0 )
  {
    current_millis = millis();
    while(millis()-current_millis<1500)
    {
      ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
    }
    ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
    if(fabs(Old_Yaw-Yaw)<0.05 && fabs(Old_Pitch-Pitch)<0.05)
    {
      Initial_Yaw=Yaw;
      Initial_Pitch=Pitch;
    }
    Old_Yaw = Yaw;
    Old_Pitch = Pitch;
  }


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

     
  

}

void Stop_P_Window()
{
  if(digitalRead(LIMIT_SW_PIN)==0)
  {
    ledcWrite(p_window_pwm_channel,0);
  }
}

void loop()
{
  while (ps5.isConnected() == true) 
  {
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
        Serial.print("Yaw:\t");
        Serial.print(Yaw_difference);
        Serial.print("\xC2\xB0"); //Print degree symbol
        Serial.println();

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
      myservo1.write(72); // rotate the servo to 180 degrees to close
      myservo2.write(80);
      delay(300);
      myservo1.write(76); // rotate the servo to 180 degrees to close
      myservo2.write(76); // rotate the servo to 180 degrees to close
      current_millis = millis();
      while(millis()-current_millis<500)
      {
        vesc1.setRPM(rpm);
      }
      myservo3.write(90);
      current_millis = millis();
      while(millis()-current_millis<500)
      {
        vesc1.setRPM(rpm);
      }
      myservo3.write(190);
    }
    
    if (ps5.Circle() )
    {
      if(Circle_Pressed == false)
      {
        Circle_Pressed = true;
        if(Grip_Servo ==false)
        {
          Grip_Servo = true;
          myservo1.write(76); // rotate the servo to 180 degrees to close
          myservo2.write(76); // rotate the servo to 180 degrees to close
        }
        else
        {
          Grip_Servo = false;
          myservo1.write(66); // rotate the servo to 0 degrees to open
          myservo2.write(86);
        }
      }
    }
    else
    {
      Circle_Pressed = false;
    }

    if (ps5.R1())
    {
      digitalWrite(P_WINDOW_DIR_PIN,HIGH);
      ledcWrite(p_window_pwm_channel,120);
    }
    else if(ps5.R2()&&(digitalRead(LIMIT_SW_PIN)==HIGH))//
    {
      digitalWrite(P_WINDOW_DIR_PIN,LOW);
      ledcWrite(p_window_pwm_channel,120);
    }
    else
    {
      ledcWrite(p_window_pwm_channel,0);
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

    if (ps5.Left())
    {
      Aim_successful = false;
      DJI_Increase_Speed = -3;
      Magnitude_L=90;
      Angle_L = 180;

      while(Aim_successful == false)
      {
        if (ps5.Cross())
        {
          for(i = 0; i<4 ; i++)
          {
            pulse_M[i] = default_M[i];
            ledcWrite(i+3,pulse_M[i]);
          }
          break;
        }
        Lidar_Distance = Get_Lidar_data();
        if(Lidar_Distance > POLE_DISTANCE)
        {
          
          calc_robot_dir ( &Magnitude_L , &Angle_L, pulse_M , default_M,Motor_dir,DJI_Increase_Speed);

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

          for(i=0;i<4;i++)
          {
          Setpoint_Adjust_Yaw (Yaw_difference, i,Motor_dir[i], pulse_M , SPEED_CHG_PER_YAW );
          }

          for(i = 0; i < 4; i++)
          {
            ledcWrite(i+3,pulse_M[i]);
          }
      
        }
        else
        {
          for(i = 0; i < 4; i++)
          {
            ledcWrite(i+3,default_M[i]);
          }
          Aim_successful=true;
          break;
        }
      }
    }

    if(ps5.Right())
    {
      Target_Pitch = 10;
      ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
      Pitch_difference = Initial_Pitch - Pitch ;

      while (fabs(Pitch_difference - Target_Pitch) > 1 ) 
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

    if (ps5.L2())
    {
      if (L2_Pressed == false)
      {
        L2_Pressed = true;
        DJI_Increase_Speed += 1;
        if(DJI_Increase_Speed> 10)
        {
          DJI_Increase_Speed=10;
        }
      }
    }
    else
    {
      L2_Pressed = false;
    }

    if (ps5.L1())
    {
      if(L1_Pressed == false)
      {
        L1_Pressed = true;
        DJI_Increase_Speed -=1;
        if(DJI_Increase_Speed<-3)
        {
          DJI_Increase_Speed = -3;
        }
      } 
    }
    else
    {
      L1_Pressed = false;
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

    // if(ps5.Circle())
    // {
    //   Rotate_angle = 90; //because if rotate clockwise is negative angle 
    // }
    // else
    // {
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
        pulse_M[i] = default_M[i]-7;
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
	

      

    while(Rotate_angle!=0 )
    {
      if (ps5.Cross())
      {
        Rotate_angle=0;
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
      if(fabs(Rotate_angle - Yaw_difference)>1)
      {
        if(Rotate_angle - Yaw_difference>0)
        {
          for(i = 0; i<4 ; i++)
          {
          pulse_M[i] = default_M[i]+7;
          ledcWrite(i+3,pulse_M[i]);
          }
        }
        else
        {
          for(i = 0; i<4 ; i++)
          {
          pulse_M[i] = default_M[i]-7;
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
        Rotate_angle = 0;
        ICM20948_GET_READING_QUAT6(&Yaw, &Pitch);
        Initial_Yaw = Yaw;
      }

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
}

