#include <ps5Controller.h>
#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <Robocon2023.h>
//#define USE_SPI       // Uncomment this to use SPI




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

  

  while (!(Pitch_difference > 45 && Pitch_difference < 45.9)) {
     ICM20948_GET_READING_QUAT6(&Pitch);
        Serial.print("Pitch_reading:\t");
        Serial.print(Pitch);
        Serial.print("\xC2\xB0"); //Print degree symbol
        Serial.println();
        Pitch_difference = Initial_Pitch - Pitch ;
        Serial.print("Pitch diff: ");
        Serial.print(Pitch_difference);
                Serial.print("\xC2\xB0"); //Print degree symbol
        Serial.println();
  }
    ledcWrite(l_actuator_pwm_channel,0);

}



if(ps5.Triangle()){
while(Pitch !=0.98 || Pitch != -0.98 || Pitch != 0 ){
      digitalWrite(L_ACTUATOR_DIR_PIN,LOW);
      ledcWrite(l_actuator_pwm_channel,80);
    }

  }

}



}


    

