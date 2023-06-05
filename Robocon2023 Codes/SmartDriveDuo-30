#include <Cytron_SmartDriveDuo.h>
#include <ps5Controller.h>
#include "Arduino.h"

#define LP_WINDOW_PWM_PIN 25 // AN1
#define LP_WINDOW_DIR_PIN 12 // IN1 - Clockwise (HIGH)
#define RP_WINDOW_PWM_PIN 26 // AN2
#define RP_WINDOW_DIR_PIN 13 // IN2 - Counter-Clockwise (LOW)
Cytron_SmartDriveDuo smartDriveDuo30(PWM_INDEPENDENT, LP_WINDOW_DIR_PIN, RP_WINDOW_DIR_PIN, LP_WINDOW_PWM_PIN, RP_WINDOW_PWM_PIN);

int lp_window_pwm_channel = 3;
int rp_window_pwm_channel = 4;
int frequency_actuator = 500;
int resolution_actuator = 8;

void setup() { 

  ledcSetup(lp_window_pwm_channel, frequency_actuator, resolution_actuator); //channel 0
  ledcSetup(rp_window_pwm_channel, frequency_actuator, resolution_actuator); //channel 1

  Serial.begin(115200);

  ledcAttachPin(LP_WINDOW_PWM_PIN, lp_window_pwm_channel);
  ledcAttachPin(RP_WINDOW_PWM_PIN, rp_window_pwm_channel);\

  ps5.begin("00:BE:3B:F7:2D:51"); //WHITE CONTROLLER
  //ps5.begin("1C:15:1F:41:39:0A"); //BLACK CONTROLLER
  Serial.println("Ready.");
}

void loop() {
  while (Serial.available() == 0) {
  }

  int menuChoice = Serial.parseInt();

  switch (menuChoice) {
    case 1:
      // temp sensor code goes here
      Serial.print("Going Down");
      digitalWrite(LP_WINDOW_DIR_PIN, HIGH);
      ledcWrite(lp_window_pwm_channel, 60);
      digitalWrite(RP_WINDOW_DIR_PIN, HIGH);
      ledcWrite(rp_window_pwm_channel, 60);
      break;

    case 2:
      // humidity sensor code goes here
      Serial.print("Going Up");
      digitalWrite(LP_WINDOW_DIR_PIN, LOW);
      ledcWrite(lp_window_pwm_channel, 60);
      digitalWrite(RP_WINDOW_DIR_PIN, LOW);
      ledcWrite(rp_window_pwm_channel, 60);
      break;

    case 3:
      Serial.print("Stopping");
      ledcWrite(lp_window_pwm_channel, 0);
      ledcWrite(rp_window_pwm_channel, 0);
      break;

    default:
      Serial.println("Please choose a valid selection");
  }



  // if(ps5.isConnected() != true){
  //   Serial.println("Not Connected");
  //   delay(2000);
  // }
  // else{
  //   Serial.println("Connected");
  //   delay(2000);
  // }
  

  // int rotate = 0;
  // rotate += 1;
  // if (rotate < 10)
  //   {
  //     digitalWrite(LP_WINDOW_DIR_PIN, LOW);
  //     ledcWrite(lp_window_pwm_channel, 60);
  //     digitalWrite(RP_WINDOW_DIR_PIN, HIGH);
  //     ledcWrite(rp_window_pwm_channel, 60);
  //   }
  //   else if(rotate > 20 && rotate < 20)//
  //   {
  //     digitalWrite(LP_WINDOW_DIR_PIN, HIGH);
  //     ledcWrite(lp_window_pwm_channel, 60);
  //     digitalWrite(RP_WINDOW_DIR_PIN, LOW);
  //     ledcWrite(rp_window_pwm_channel, 60);
  //   }
  //   else
  //   {
  //     ledcWrite(lp_window_pwm_channel, 0);
  //     ledcWrite(rp_window_pwm_channel, 0);
  //   }
    

  // while (ps5.isConnected() == true) 
  // {
  //   if (ps5.R1())
  //   {
  //     digitalWrite(LP_WINDOW_DIR_PIN, LOW);
  //     ledcWrite(lp_window_pwm_channel, 60);
  //     digitalWrite(RP_WINDOW_DIR_PIN, HIGH);
  //     ledcWrite(rp_window_pwm_channel, 60);
  //   }
  //   else if(ps5.R2())//
  //   {
  //     digitalWrite(LP_WINDOW_DIR_PIN, HIGH);
  //     ledcWrite(lp_window_pwm_channel, 60);
  //     digitalWrite(RP_WINDOW_DIR_PIN, LOW);
  //     ledcWrite(rp_window_pwm_channel, 60);
  //   }
  //   else
  //   {
  //     ledcWrite(lp_window_pwm_channel, 0);
  //     ledcWrite(rp_window_pwm_channel, 0);
  //   }
  // }
}
