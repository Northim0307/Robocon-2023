#include <mcp_can.h>
#include <SPI.h>
#include <PID_v1.h>
#include <ps5Controller.h>
#include "Arduino.h"
#include <Robocon2023.h>

#define OPTIMUM_SPEED_M1 750
#define OPTIMUM_SPEED_M2 750
#define OPTIMUM_SPEED_M3 750
#define OPTIMUM_SPEED_M4 750

//test on table
#define MAX_CURRENT_M1 1000 //max 16384      
#define MIN_CURRENT_M1 0
#define MAX_CURRENT_M2 1000 //max 16384
#define MIN_CURRENT_M2 0
#define MAX_CURRENT_M3 1000 //max 16384
#define MIN_CURRENT_M3 0
#define MAX_CURRENT_M4 1000 //max 16384
#define MIN_CURRENT_M4 0

//test on floor
//#define MAX_CURRENT_M1 2000 //max 16384      //1000 if test on table
//#define MIN_CURRENT_M1 0
//#define MAX_CURRENT_M2 2000 //max 16384
//#define MIN_CURRENT_M2 0
//#define MAX_CURRENT_M3 2000 //max 16384
//#define MIN_CURRENT_M3 0
//#define MAX_CURRENT_M4 2000 //max 16384
//#define MIN_CURRENT_M4 0


//test on table
#define DEFAULT_STARTING_CURRENT 1000          //1000 if test on table

//test on floor
//#define DEFAULT_STARTING_CURRENT 2000          //1000 if test on table




////test on table
const double OPTIMUM_CLOCKWISE_CURRENT_M[]= {200,200,200,200};         //200 if test on table
const double OPTIMUM_COUNTER_CLOCKWISE_CURRENT_M[]= {200,200,200,200};

//test on floor
//const double OPTIMUM_CLOCKWISE_CURRENT_M[]= {900,900,900,900};         //200 if test on table
//const double OPTIMUM_COUNTER_CLOCKWISE_CURRENT_M[]= {900,900,900,900};

int Robot_dir_now;
int Robot_dir_prev=STOP;
bool Robot_dir_chged;
bool starting_motor[4]= {true,true,true,true};
double CAN_Output_M[4];
int Motor_dir[4];

int Sample_Size = 1000;
double Optimum_Current[5][4]={0};
              

/*CAN BUS variables */

char msgString[128];          //An array of char to store the msg string for printing 
uint16_t feedback_M[4];


/*PID variables */

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


bool PID_completed;
int retry;
int flag_motor1,flag_motor2,flag_motor3,flag_motor4;  //flags to indicate whether the feedback for both motor are received

void setup()
{
  Serial.begin(115200);
  ps5.begin("00:BE:3B:F7:2D:51");
  //ps5.begin("D8:9E:61:A0:93:74"); //replace with your MAC address
  Serial.println("Ready.");
  
  // Initialize MCP2515 running at 8MHz with a baudrate of 1Mb/s and the masks and filters disabled.
  CAN0.begin(MCP_ANY, CAN_1000KBPS,MCP_8MHZ);
   
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
  can_output.can_id = 0x200;
  can_output.can_dlc = 8;

  //Initial values for PID
  Input_M[0] = Input_M[1] = Input_M[2] = Input_M[3] = 0;

  Setpoint_M[0]= OPTIMUM_SPEED_M1 ; //RPM
  Setpoint_M[1]= OPTIMUM_SPEED_M2 ; //RPM
  Setpoint_M[2]= OPTIMUM_SPEED_M3 ; //RPM
  Setpoint_M[3]= OPTIMUM_SPEED_M4 ; //RPM

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
  while (ps5.isConnected() == true) {
  if (ps5.Cross())
    {
      Robot_Direction(Robot_dir_now = STOP, Motor_dir);
      CAN_Output_M[0] = CAN_Output_M[1] = CAN_Output_M[2] = CAN_Output_M[3] = 0;
    }
    else
    {
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

//  Serial.print(" move_command now:  ");
//  Serial.println(move_command);
  if( Robot_dir_now != STOP) 
    {
      // Obtain feedback from both motor
      CAN_receive_msg(feedback_M);

      Input_M[0]= feedback_M[0];
      Input_M[1]= feedback_M[1];
      Input_M[2]= feedback_M[2];
      Input_M[3]= feedback_M[3];
    
      PrintFeedback(feedback_M);

      if (Robot_dir_prev != Robot_dir_now)
      {
        Robot_dir_chged = true;
        
        PID_M1.SetMode(MANUAL);
        PID_M2.SetMode(MANUAL);
        PID_M3.SetMode(MANUAL);
        PID_M4.SetMode(MANUAL);
        
        starting_motor[0] = starting_motor[1] = starting_motor[2] = starting_motor[3] = true;
        Output_M[0] = Output_M[1] = Output_M[2] = Output_M[3] = 0;
      }
      
      Robot_dir_prev = Robot_dir_now;

  if(Robot_dir_chged == true)
      {
        if((feedback_M[0]==0)&&
           (feedback_M[1]==0)&&
           (feedback_M[2]==0)&&
           (feedback_M[3]==0))
           {
            Robot_dir_chged = false;
           }
      }
  
      if(Robot_dir_chged == false)
      {
        for(i=0;i<4;i++)
        {
          if(starting_motor[i] && (feedback_M[i] < 250))
          {
            Output_M[i] = DEFAULT_STARTING_CURRENT;
          }
          else
          {
            if(starting_motor[i]==true)
            {
              starting_motor[i]=false;
              if(Motor_dir[i] == CLOCKWISE)
              {
                Output_M[i] = OPTIMUM_CLOCKWISE_CURRENT_M[i]; 
              }
              else if(Motor_dir[i] == COUNTER_CLOCKWISE) 
              {
                Output_M[i] = OPTIMUM_COUNTER_CLOCKWISE_CURRENT_M[i]; 
              }
            }
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
    }
  }
  }


//   PID_completed = false;
//   while(PID_completed==false)
//   {
//     PID_completed = (PID_M1.Compute()&&PID_M2.Compute()&&PID_M3.Compute()&&PID_M4.Compute());
//        }


    for(i = 0 ; i < 4 ; i++)
    {
      if (Motor_dir[i]== COUNTER_CLOCKWISE)              
      {
        CAN_Output_M[i] = Output_M[i] ;
        //Output_M[i] = Output_M[i] ;       //positive is counterclockwise
      }
      else
      {
        CAN_Output_M[i] = (-Output_M[i]);
        //Output_M[i] = (-Output_M[i]);
      }
    }
  }

//  if(move_command == MOVING)
//  {
//  Serial.print("Output: ");
//  for (i = 0 ; i< 4 ;i++)
//  {
//    Serial.print(CAN_Output_M[i]);
//  }
//  Serial.println();
//  }
 
  
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
  if(sndStat == CAN_OK){
//    Serial.println("Message Sent Successfully!");
  } else {
    Serial.println("Error Sending Message...");
  }

 


//  Serial.println();
  

  // started=true;
}
}
