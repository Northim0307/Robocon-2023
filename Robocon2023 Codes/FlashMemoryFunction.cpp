float angle;
int motorspeed;

void saveData(float angle, int motorSpeed) {
  const int angleAddr = 0;
  

  byte* anglePtr = (byte*)(&angle);
  for (int i = 0; i < sizeof(float); i++) {
    EEPROM.write(angleAddr + i, anglePtr[i]);
  }
  
  const int motorSpeedAddr = sizeof(float) + angleAddr; 
  byte* motorSpeedPtr = (byte*)(&motorSpeed);
  for (int i = 0; i < sizeof(int); i++) {
    EEPROM.write(motorSpeedAddr + i, motorSpeedPtr[i]);
  }

  EEPROM.commit();
}


void readData(float &angle, int &motorSpeed) {
  const int angleAddr = 0;

  byte* anglePtr = (byte*)(&angle);
  for (int i = 0; i < sizeof(float); i++) {
    anglePtr[i] = EEPROM.read(angleAddr + i);
  }

  const int motorSpeedAddr = sizeof(float) + angleAddr; 
  byte* motorSpeedPtr = (byte*)(&motorSpeed);
  for (int i = 0; i < sizeof(int); i++) {
    motorSpeedPtr[i] = EEPROM.read(motorSpeedAddr + i);
  }
}

/*later when you want to retrieve the saved data, just call the function readData(angle, int motorSpeed)
 it then will passed the data into angle and motorspeed variable.
*/
