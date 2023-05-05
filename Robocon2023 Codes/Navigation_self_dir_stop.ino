#include <mcp_can.h>
#include <SPI.h>
#include <PID_v1.h>
#include <ps5Controller.h>
#include "Arduino.h"
// #include <MPU9250.h>
#include <EEPROM.h>
#include <ICM_20948.h>
#include <Robocon2023.h>





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
#define MAX_CURRENT_M1 4000 //max 16384      //1000 if test on table
#define MIN_CURRENT_M1 0
#define MAX_CURRENT_M2 4000 //max 16384
#define MIN_CURRENT_M2 0
#define MAX_CURRENT_M3 4000 //max 16384
#define MIN_CURRENT_M3 0
#define MAX_CURRENT_M4 4000 //max 16384
#define MIN_CURRENT_M4 0


//test on table
//#define DEFAULT_STARTING_CURRENT 1000          //1000 if test on table

//test on floor
#define DEFAULT_STARTING_CURRENT 3000          //1000 if test on table

//MPU9250
#define MAGNETIC_DECLINATION -0.78 // To be defined by user

#define SPEED_CHG_PER_YAW 5

const int OPTIMUM_SPEED_M[]= {1000,1000,1000,1000};         //200 if test on table

double Initial_Yaw=0;
double Old_Yaw, Yaw =0, Yaw_difference =0;
unsigned long current_millis;


////test on table
//const double OPTIMUM_CLOCKWISE_CURRENT_M[]= {200,200,200,200};         //200 if test on table
//const double OPTIMUM_COUNTER_CLOCKWISE_CURRENT_M[]= {200,200,200,200};

//test on floor
const double OPTIMUM_CLOCKWISE_CURRENT_M[]= {1100,1100,1100,1100};         //200 if test on table
const double OPTIMUM_COUNTER_CLOCKWISE_CURRENT_M[]= {1100,1100,1100,1100};

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

bool PID_completed = false;

//Define Variables and initial tuning parameters for PID
double Setpoint_M[4], Input_M[4], Output_M[4]; 
double Kp_M1=0.5, Ki_M1=1.0, Kd_M1=0.00;       //Kp,Ki,Kd for motor 1
double Kp_M2=0.5, Ki_M2=1.0, Kd_M2=0.00;       //Kp,Ki,Kd for motor 2
double Kp_M3=0.5, Ki_M3=1.0, Kd_M3=0.00;       //Kp,Ki,Kd for motor 1
double Kp_M4=0.5, Ki_M4=1.0, Kd_M4=0.00;       //Kp,Ki,Kd for motor 2

//Specify the links and Initiate the PID
PID PID_M1(&Input_M[0], &Output_M[0], &Setpoint_M[0], Kp_M1, Ki_M1, Kd_M1,0, DIRECT);
PID PID_M2(&Input_M[1], &Output_M[1], &Setpoint_M[1], Kp_M2, Ki_M2, Kd_M2,0, DIRECT);
PID PID_M3(&Input_M[2], &Output_M[2], &Setpoint_M[2], Kp_M3, Ki_M3, Kd_M3,0, DIRECT);
PID PID_M4(&Input_M[3], &Output_M[3], &Setpoint_M[3], Kp_M4, Ki_M4, Kd_M4,0, DIRECT);

float Magnitude, Angle;

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

  // Initialize MCP2515 running at 8MHz with a baudrate of 1Mb/s and the masks and filters disabled.
  CAN0.begin(MCP_ANY, CAN_1000KBPS,MCP_8MHZ);
   
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  can_output.can_id = 0x200;
  can_output.can_dlc = 8;

  //Initial values for PID
  Input_M[0] = Input_M[1] = Input_M[2] = Input_M[3] = 0;

  Setpoint_M[0]= OPTIMUM_SPEED_M[0] ; //RPM
  Setpoint_M[1]= OPTIMUM_SPEED_M[1] ; //RPM
  Setpoint_M[2]= OPTIMUM_SPEED_M[2] ; //RPM
  Setpoint_M[3]= OPTIMUM_SPEED_M[3] ; //RPM

  PID_M1.SetMode(MANUAL);
  PID_M1.SetOutputLimits(MIN_CURRENT_M1, MAX_CURRENT_M1);
  PID_M2.SetMode(MANUAL);
  PID_M2.SetOutputLimits(MIN_CURRENT_M2, MAX_CURRENT_M2);
  PID_M3.SetMode(MANUAL);
  PID_M3.SetOutputLimits(MIN_CURRENT_M3, MAX_CURRENT_M3);
  PID_M4.SetMode(MANUAL);
  PID_M4.SetOutputLimits(MIN_CURRENT_M4, MAX_CURRENT_M4);

  

}


void loop()
{
  while (ps5.isConnected() == true) 
  {
 
    if (ps5.Cross())
    {
      Robot_Direction(Robot_dir_now = STOP, Motor_dir);
    }
    else
    {
      Analog_Stick_Calc(float(ps5.LStickX()),float(ps5.LStickY()), &Magnitude , &Angle );
      calc_robot_dir (Prev_Motor_dir, Starting_Motor, &Magnitude , &Angle, Motor_dir, Setpoint_M , OPTIMUM_SPEED_M);
      if (ps5.Up())
      {
        Robot_Direction(Robot_dir_now = FORWARD, Motor_dir);
      }
      else if (ps5.Down())
      {
        Robot_Direction(Robot_dir_now = BACKWARD, Motor_dir);
      }
      else if (ps5.Left())
      {
        Robot_Direction(Robot_dir_now = LEFTWARD, Motor_dir);
      }
      else if (ps5.Right())
      {
        Robot_Direction(Robot_dir_now = RIGHTWARD,Motor_dir);
      }
      else if (ps5.Circle())
      {
        Robot_Direction(Robot_dir_now = ROTATE_CW,Motor_dir);
      }
      else if (ps5.Square())
      {
       Robot_Direction(Robot_dir_now = ROTATE_C_CW,Motor_dir);
      }
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
      

    
    if( Magnitude > 30) 
    {
      // Obtain feedback from both motor
      CAN_receive_msg(feedback_M);

      Input_M[0]= feedback_M[0];
      Input_M[1]= feedback_M[1];
      Input_M[2]= feedback_M[2];
      Input_M[3]= feedback_M[3];
    
      PrintFeedback(feedback_M);


        
        for(i=0;i<4;i++)
        {
          if(Starting_Motor[i])
          {
            switch(i)
              {
                case 0 :
                PID_M1.SetMode(MANUAL);
                break;    
                
                case 1 :
                PID_M2.SetMode(MANUAL);
                break; 
                
                case 2 :
                PID_M3.SetMode(MANUAL);
                break;  
                
                case 3 :
                PID_M4.SetMode(MANUAL);
                break;   
              }
            Output_M[i] = 0;
          }
        }

        // for(i=0;i<4;i++)
        // {

        // if(Starting_Motor[i]==true && feedback_M[i] == 0)
        // {
        //   Starting_Motor[i] = false;
        // }
        // }
      
    
      // if(Robot_dir_chged == true)
      // {
      //   if((feedback_M[0]==0)&&
      //      (feedback_M[1]==0)&&
      //      (feedback_M[2]==0)&&
      //      (feedback_M[3]==0))
      //      {
      //       Robot_dir_chged = false;
      //      }
      // }
  

        for(i=0;i<4;i++)
        {
          if(Starting_Motor[i] && (feedback_M[i] < 250))
          {
            Output_M[i] = DEFAULT_STARTING_CURRENT;
          }
          else
          {
            if(Starting_Motor[i]==true)
            {
              Starting_Motor[i]=false;
              if(Motor_dir[i] == CLOCKWISE)
              {
                Output_M[i] = OPTIMUM_CLOCKWISE_CURRENT_M[i]; 
              }
              else if(Motor_dir[i] == COUNTER_CLOCKWISE) 
              {
                Output_M[i] = OPTIMUM_COUNTER_CLOCKWISE_CURRENT_M[i]; 
              }
            }
			
			      Setpoint_Adjust_Yaw (Yaw_difference, i,Motor_dir[i], Setpoint_M , OPTIMUM_SPEED_M[i], SPEED_CHG_PER_YAW );

            Serial.print("Setpoint of Motor:");
            Serial.print(i);
            Serial.print("is: ");
            Serial.println (Setpoint_M[i]);

            PID_completed = false;
            while(PID_completed==false)
            {
              switch(i)
              {
                case 0 :
                PID_M1.SetMode(AUTOMATIC);
                PID_completed = PID_M1.Compute();
                break;    
                
                case 1 :
                PID_M2.SetMode(AUTOMATIC);
                PID_completed = PID_M2.Compute();
                break; 
                
                case 2 :
                PID_M3.SetMode(AUTOMATIC);
                PID_completed = PID_M3.Compute();
                break;  
                
                case 3 :
                PID_M4.SetMode(AUTOMATIC);
                PID_completed = PID_M4.Compute();
                break;   
              }
            }
            
//            if(Optimum_Current[Robot_dir_now][i] == 0)
//            {
//              Optimum_Current[Robot_dir_now][i] = calc_optimum_current ( Output_M[i], Sample_Size, Robot_dir_now, i);
//            }
//            else
//            {
//              Serial.print(" Optimum Current of Motor : ");
//              Serial.print(i+1);
//              Serial.print(" in direction of :  ");  
//              Serial.print(Robot_dir_now);
//              Serial.print(" is :");
//              Serial.println(Optimum_Current[Robot_dir_now][i]);
//            }
            
          }
        }
      
    }
    else
    {
      Output_M[0] = Output_M[1] = Output_M[2] = Output_M[3] = 0;
    }

//    Serial.print("Output: ");
//  for (i = 0 ; i< 4 ;i++)
//  {
//    Serial.print("Motor_Direction of motor:");
//    Serial.print(i);
//    Serial.print("is: ");
//    Serial.print(Motor_dir[i]);
//    Serial.print(" with output of : ");
//    Serial.println(Output_M[i]);
//  }
  Serial.println();
    CAN_send_msg(Motor_dir,Output_M);
  }
}
