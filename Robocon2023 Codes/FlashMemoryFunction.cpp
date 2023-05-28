#include <EEPROM.h>

float angle = 46.67;
int motorSpeed = 123;

float savedAngle;
int savedMotorSpeed;

void saveData() {
  EEPROM.write(0, angle);
  EEPROM.write(1, motorSpeed);
  EEPROM.commit();
}

void readData() {
  savedAngle = EEPROM.read(0);
  savedMotorSpeed = EEPROM.read(1);
}

void setup() {
  Serial.begin(115200);

  EEPROM.begin(512);

  saveData();
}

void loop() {
  readData();

  Serial.print("Saved Angle: ");
  Serial.println(savedAngle);
  Serial.print("Saved Motor speed: ");
  Serial.println(savedMotorSpeed);

  delay(1000);
}
