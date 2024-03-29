// #include <mcp_can.h>
// #include <SPI.h>
#include <PID_v1.h>
#include <ps5Controller.h>
#include "Arduino.h"
// #include <MPU9250.h>
#include <EEPROM.h>
#include <ICM_20948.h>
#include <Robocon2023.h>
#include <VescUart.h>

#define SERVO_1_PIN  12
#define SERVO_2_PIN  14
#define SERVO_3_PIN  2
#define LIMIT_SW_PIN 4

#define L_ACTUATOR_PWM_PIN 27
#define L_ACTUATOR_DIR_PIN 26

#define P_WINDOW_PWM_PIN 33
#define P_WINDOW_DIR_PIN 25

#define MOTOR1_PIN 5
#define MOTOR2_PIN 18
#define MOTOR3_PIN 19
#define MOTOR4_PIN 23

#define MOTOR1_STARTING 184
#define MOTOR2_STARTING 188
#define MOTOR3_STARTING 190
#define MOTOR4_STARTING 184


#define SPEED_CHG_PER_YAW 1

#define DEFAULT_FLIPSKY_RPM 10500
#define FLIPSKY_HIGHEST_RPM 24500
#define FLIPSKY_LOWEST_RPM 3500


VescUart vesc1;

int motor1_pwm_channel     = 0;
int motor2_pwm_channel     = 1;
int motor3_pwm_channel     = 2;
int motor4_pwm_channel     = 3;
int l_actuator_pwm_channel = 4;
int p_window_pwm_channel   = 5;
int grip_servo1_pwm_channel = 6;
int grip_servo2_pwm_channel = 7;
int push_servo_pwm_channel = 8;

int frequency_dji = 500;
int resolution_dji = 8; //max is 255

int frequency_actuator     = 1000;
int resolution_actuator = 8; //max is 255

int frequency_servo     = 50;
int resolution_servo = 16; //max is 65535


// int resolution_dji = 16; //max is 65535

// const int OPTIMUM_SPEED_M[]= {1000,1000,1000,1000};         //200 if test on table

double Initial_Yaw=0;
double Old_Yaw, Yaw =0, Yaw_difference =0;
unsigned long current_millis;


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

long push_servo_angle,push_servo_pwm;
long grip_servo1_angle,grip_servo1_pwm;
long grip_servo2_angle,grip_servo2_pwm;

  


void setup()
{
  ledcSetup(motor1_pwm_channel,frequency_dji,resolution_dji);                //channel 0
  ledcSetup(motor2_pwm_channel,frequency_dji,resolution_dji);                //channel 1
  ledcSetup(motor3_pwm_channel,frequency_dji,resolution_dji);                //channel 2
  ledcSetup(motor4_pwm_channel,frequency_dji,resolution_dji);                //channel 3
  ledcSetup(l_actuator_pwm_channel,frequency_actuator,resolution_actuator);  //channel 4
  ledcSetup(p_window_pwm_channel,frequency_actuator,resolution_actuator);    //channel 5
  ledcSetup(grip_servo1_pwm_channel,frequency_servo,resolution_servo);       //channel 6
  ledcSetup(grip_servo2_pwm_channel,frequency_servo,resolution_servo);       //channel 7
  ledcSetup(push_servo_pwm_channel,frequency_servo,resolution_servo);        //channel 8



  Serial.begin(115200);
  Serial2.begin(115200); //serial1

  vesc1.setSerialPort(&Serial2); //&serial

  pinMode(LIMIT_SW_PIN, INPUT);
  pinMode(L_ACTUATOR_DIR_PIN,OUTPUT);
  pinMode(P_WINDOW_DIR_PIN,OUTPUT);
  



  ledcAttachPin(MOTOR1_PIN,motor1_pwm_channel);
  ledcAttachPin(MOTOR2_PIN,motor2_pwm_channel);
  ledcAttachPin(MOTOR3_PIN,motor3_pwm_channel);
  ledcAttachPin(MOTOR4_PIN,motor4_pwm_channel);
  ledcAttachPin(L_ACTUATOR_PWM_PIN,l_actuator_pwm_channel);
  ledcAttachPin(P_WINDOW_PWM_PIN,p_window_pwm_channel);
  ledcAttachPin(SERVO_1_PIN,grip_servo1_pwm_channel);
  ledcAttachPin(SERVO_2_PIN,grip_servo2_pwm_channel);
  ledcAttachPin(SERVO_3_PIN,push_servo_pwm_channel);
  
  attachInterrupt(digitalPinToInterrupt(LIMIT_SW_PIN),Stop_P_Window,RISING);

  ps5.begin("00:BE:3B:F7:2D:51");
  //ps5.begin("D8:9E:61:A0:93:74"); //replace with your MAC address
  Serial.println("Ready.");

  ICM20948_SETUP_QUAT6();

  while(Initial_Yaw==0)
  {
    current_millis = millis();
    while(millis()-current_millis<1500)
    {
      ICM20948_GET_READING_QUAT6(&Yaw);
    }
    ICM20948_GET_READING_QUAT6(&Yaw);
    if(fabs(Old_Yaw-Yaw)<0.05)
    {
      Initial_Yaw=Yaw;
      digitalWrite(2,LOW);
    }
    Old_Yaw=Yaw;
  }


  Serial.print("Initial Yaw: ");
  Serial.println(Initial_Yaw);

  

  

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
  if(digitalRead(LIMIT_SW_PIN))
  {
    ledcWrite(p_window_pwm_channel,0);
  }
  return;

}

void loop()
{
  while (ps5.isConnected() == true) 
  {
    vesc1.setRPM(rpm);
        ICM20948_GET_READING_QUAT6(&Yaw);
        Serial.print("Yaw_reading:\t");
        Serial.print(Yaw);
        Serial.print("\xC2\xB0"); //Print degree symbol
        Serial.println();
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
      ICM20948_GET_READING_QUAT6(&Yaw);
      Initial_Yaw = Yaw;
      Serial.print("Initial Yaw: ");
      Serial.println(Initial_Yaw);

    }

    if (ps5.Square())
    {
      rpm = DEFAULT_FLIPSKY_RPM;
      vesc1.setRPM(rpm);
    } 

    if (ps5.Triangle())   //push servo
    {
      push_servo_angle = 35;
      push_servo_pwm = map(push_servo_angle,0,180,204,1024);
      ledcWrite(push_servo_pwm_channel,push_servo_pwm);
      delay(500);
      push_servo_angle = 155;
      push_servo_pwm = map(push_servo_angle,0,180,204,1024);
      ledcWrite(push_servo_pwm_channel,push_servo_pwm);
    }
    
    if (ps5.R1())
    {
      grip_servo1_angle = 5;
      grip_servo1_pwm = map(grip_servo1_angle,0,270,204,1024);
      ledcWrite(grip_servo1_pwm_channel,grip_servo1_pwm);
      grip_servo2_angle = 136;
      grip_servo2_pwm = map(grip_servo2_angle,0,270,204,1024);
      ledcWrite(grip_servo2_pwm_channel,grip_servo2_pwm);
    }
    if (ps5.R2())
      {
        grip_servo1_angle = 32;
        grip_servo1_pwm = map(grip_servo1_angle,0,270,204,1024);
        ledcWrite(grip_servo1_pwm_channel,grip_servo1_pwm);
        grip_servo2_angle = 98;
        grip_servo2_pwm = map(grip_servo2_angle,0,270,204,1024);
        ledcWrite(grip_servo2_pwm_channel,grip_servo2_pwm);

      }
    

    if (ps5.L1())
    {
      digitalWrite(P_WINDOW_DIR_PIN,HIGH);
      ledcWrite(p_window_pwm_channel,80);
    }
    else if(ps5.L2())
    {
      digitalWrite(P_WINDOW_DIR_PIN,LOW);
      ledcWrite(p_window_pwm_channel,80);
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
 

    if (ps5.Cross())
    {
      rpm = 0;
      vesc1.setRPM(rpm);

      for(i = 0; i < 4; i++)
      {
        pulse_M[i]=default_M[i];
        ledcWrite(i,pulse_M[i]);
      }
      ledcWrite(l_actuator_pwm_channel,0);
      ledcWrite(p_window_pwm_channel,0);
      
    }

    if(ps5.Circle())
    {
      Rotate_angle = 90; //because if rotate clockwise is negative angle 
    }
    // else
    // {
      Analog_Stick_Calc(float(ps5.LStickX()),float(ps5.LStickY()), &Magnitude_L , &Angle_L );
      Analog_Stick_Calc(float(ps5.RStickX()),float(ps5.RStickY()), &Magnitude_R , &Angle_R );
      calc_robot_dir ( &Magnitude_L , &Angle_L, pulse_M , default_M,Motor_dir);
    // }

    if(Magnitude_R>40)
    {
      if((Angle_R>-30 && Angle_R <=0)||(Angle_R>0 && Angle_R <=30))
      {
        for(i = 0;i<4;i++)
        {
        pulse_M[i] = default_M[i]+7;
        ledcWrite(i,pulse_M[i]);
        }
      }
      if((Angle_R>150 && Angle_R <=180)||(Angle_R>-180 && Angle_R <=-150))
      {
        for(i = 0;i<4;i++)
        {
        pulse_M[i] = default_M[i]-7;
        ledcWrite(i,pulse_M[i]);
        }
      }
      if((Angle_R>60 && Angle_R <=120))
      {
        rpm = rpm + 700;
        if(rpm>FLIPSKY_HIGHEST_RPM)
        {
          rpm=FLIPSKY_HIGHEST_RPM;

        }
        Serial.print("rpm_now: ");
        Serial.println(rpm);

      }
      if((Angle_R<-60 && Angle_R >=-120))
      {
        rpm = rpm - 700;
        if(rpm<FLIPSKY_LOWEST_RPM)
        {
          rpm=FLIPSKY_LOWEST_RPM;
        }
        Serial.print("rpm_now: ");
        Serial.println(rpm);

      }
    }
	

      

    while(Rotate_angle!=0 )
    {
      if (ps5.Cross())
      {
        Rotate_angle=0;
        for(i = 0; i<4 ; i++)
        {
          pulse_M[i] = default_M[i];
          ledcWrite(i,pulse_M[i]);
        }
        break;
      }
      ICM20948_GET_READING_QUAT6(&Yaw);
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
          ledcWrite(i,pulse_M[i]);
          }
        }
        else
        {
          for(i = 0; i<4 ; i++)
          {
          pulse_M[i] = default_M[i]-7;
          ledcWrite(i,pulse_M[i]);
          }
        }
      }
      else
      {
        for(i = 0; i<4 ; i++)
        {
        pulse_M[i] = default_M[i];
        ledcWrite(i,pulse_M[i]);
        }
        Rotate_angle = 0;
        ICM20948_GET_READING_QUAT6(&Yaw);
        Initial_Yaw = Yaw;
      }

    }

      
    
    if( Magnitude_L > 30) 
    {
      for(i=0;i<4;i++)
      {
      Setpoint_Adjust_Yaw (Yaw_difference, i,Motor_dir[i], pulse_M , SPEED_CHG_PER_YAW );
      Serial.print("Setpoint of Motor:");
      Serial.print(i);
      Serial.print("is: ");
      Serial.println (pulse_M[i]);
      }
      for(i = 0; i < 4; i++)
    {
        ledcWrite(i,pulse_M[i]);
    }
    }
      
    if( Magnitude_L < 30 && Magnitude_R <30) 
    {
      for(i = 0; i < 4; i++)
      {
        pulse_M[i]=default_M[i];
        ledcWrite(i,pulse_M[i]);
      }
    }
    
    
  }
}


