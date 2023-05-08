// #include <mcp_can.h>
// #include <SPI.h>
#include <PID_v1.h>
#include <ps5Controller.h>
#include "Arduino.h"
// #include <MPU9250.h>
#include <EEPROM.h>
#include <ICM_20948.h>
#include <Robocon2023.h>

#define MOTOR1_PIN 5
#define MOTOR2_PIN 18
#define MOTOR3_PIN 19
#define MOTOR4_PIN 23

#define MOTOR1_STARTING 190
#define MOTOR2_STARTING 186
#define MOTOR3_STARTING 185
#define MOTOR4_STARTING 191



//test on table
//#define MAX_CURRENT_M1 1000 //max 16384      
//#define MIN_CURRENT_M1 0
//#define MAX_CURRENT_M2 1000 //max 16384
//#define MIN_CURRENT_M2 0
//#define MAX_CURRENT_M3 1000 //max 16384
//#define MIN_CURRENT_M3 0
//#define MAX_CURRENT_M4 1000 //max 16384
//#define MIN_CURRENT_M4 0

//test on floor
// #define MAX_CURRENT_M1 4000 //max 16384      //1000 if test on table
// #define MIN_CURRENT_M1 0
// #define MAX_CURRENT_M2 4000 //max 16384
// #define MIN_CURRENT_M2 0
// #define MAX_CURRENT_M3 4000 //max 16384
// #define MIN_CURRENT_M3 0
// #define MAX_CURRENT_M4 4000 //max 16384
// #define MIN_CURRENT_M4 0


//test on table
//#define DEFAULT_STARTING_CURRENT 1000          //1000 if test on table

//test on floor
// #define DEFAULT_STARTING_CURRENT 3000          //1000 if test on table

//MPU9250
// #define MAGNETIC_DECLINATION -0.78 // To be defined by user

#define SPEED_CHG_PER_YAW 1

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

float Magnitude, Angle;

int motor1_pwm_channel = 0;
int motor2_pwm_channel = 1;
int motor3_pwm_channel = 2;
int motor4_pwm_channel = 3;


int frequency = 500;
int resolution = 8; //max is 255
// int resolution = 16; //max is 65535

int pulse_M[4];
int default_M[4]= {MOTOR1_STARTING,MOTOR2_STARTING,MOTOR3_STARTING,MOTOR4_STARTING};
float Rotate_angle;

char input;

int speed_count = 0;

void setup()
{
  Serial.begin(115200);

  pinMode(2,OUTPUT);
  digitalWrite(2,HIGH);
  ps5.begin("00:BE:3B:F7:2D:51");
  //ps5.begin("D8:9E:61:A0:93:74"); //replace with your MAC address
  Serial.println("Ready.");

  ICM20948_SETUP();

    while(Initial_Yaw==0)
  {
    current_millis = millis();
    while(millis()-current_millis<1000)
    {
      ICM20948_GET_READING(&Yaw);
    }
    ICM20948_GET_READING(&Yaw);
    if(fabs(Old_Yaw-Yaw)<0.05)
    {
      Initial_Yaw=Yaw;
      digitalWrite(2,LOW);
    }
    Old_Yaw=Yaw;
  }


  Serial.print("Initial Yaw: ");
  Serial.println(Initial_Yaw);

  ledcSetup(motor1_pwm_channel,frequency,resolution);
  ledcSetup(motor2_pwm_channel,frequency,resolution);
  ledcSetup(motor3_pwm_channel,frequency,resolution);
  ledcSetup(motor4_pwm_channel,frequency,resolution);

  ledcAttachPin(MOTOR1_PIN,motor1_pwm_channel);
  ledcAttachPin(MOTOR2_PIN,motor2_pwm_channel);
  ledcAttachPin(MOTOR3_PIN,motor3_pwm_channel);
  ledcAttachPin(MOTOR4_PIN,motor4_pwm_channel);

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


void loop()
{
  while (ps5.isConnected() == true) 
  {
 
    if (ps5.Cross())
    {
      for(i = 0; i < 4; i++)
      {
        pulse_M[i]=default_M[i];
      }
    }
    else if(ps5.Circle())
    {
      Rotate_angle = 90; 
    }
    else
    {
      Analog_Stick_Calc(float(ps5.LStickX()),float(ps5.LStickY()), &Magnitude , &Angle );
      calc_robot_dir ( &Magnitude , &Angle, pulse_M , default_M,Motor_dir);
    }
	
     ICM20948_GET_READING(&Yaw);
        Serial.print("Yaw_reading:\t");
        Serial.print(Yaw);
        Serial.print("\xC2\xB0"); //Print degree symbol
        Serial.println();
        Yaw_difference =Yaw - Initial_Yaw;
        if(Yaw_difference>=180)
        {
          Yaw_difference = -(360 - Yaw_difference);
        }
        else if (Yaw_difference<-180)
        {
          Yaw_difference = 360 + Yaw_difference;
        }
        Serial.print("Yaw:\t");
        Serial.print(Yaw_difference);
        Serial.print("\xC2\xB0"); //Print degree symbol
        Serial.println();
      

    if(Rotate_angle!=0 )
    {
      while(fabs(Rotate_angle - Yaw_difference)>1)
      {
        ICM20948_GET_READING(&Yaw);
        Yaw_difference =Yaw - Initial_Yaw;
        if(Rotate_angle + Yaw_difference>0)
        {
          for(i = 0; i<4 ; i++)
      {
        pulse_M[i] = default_M[i]+10;
      }

        }
        else
        {
          for(i = 0; i<4 ; i++)
          pulse_M[i] = default_M[i]-10;
        }


      }
    }
      else
      {
        Initial_Yaw = Yaw_difference;
        Rotate_angle = 0;
      }
      
    }
    else if( Magnitude > 30) 
    {
      for(i=0;i<4;i++)
      {
      Setpoint_Adjust_Yaw (Yaw_difference, i,Motor_dir[i], pulse_M , SPEED_CHG_PER_YAW );
      Serial.print("Setpoint of Motor:");
      Serial.print(i);
      Serial.print("is: ");
      Serial.println (pulse_M[i]);
      }
    }
      
    else
    {
      for(i = 0; i < 4; i++)
      {
        pulse_M[i]=default_M[i];
      }
    }
    for(i = 0; i < 4; i++)
    {
        ledcWrite(i,pulse_M[i]);
    }
    
  }
}

