void saveData(float angle, int motorSpeed) {
  const int angleAddr = 0;
  const int motorSpeedAddr = sizeof(float); 

  byte* anglePtr = (byte*)(&angle);
  for (int i = 0; i < sizeof(float); i++) {
    EEPROM.write(angleAddr + i, anglePtr[i]);
  }

  byte* motorSpeedPtr = (byte*)(&motorSpeed);
  for (int i = 0; i < sizeof(int); i++) {
    EEPROM.write(motorSpeedAddr + i, motorSpeedPtr[i]);
  }

  EEPROM.commit();
}


void readData(float &angle, int &motorSpeed) {
  const int angleAddr = 0;
  const int motorSpeedAddr = sizeof(float);

  byte* anglePtr = (byte*)(&angle);
  for (int i = 0; i < sizeof(float); i++) {
    anglePtr[i] = EEPROM.read(angleAddr + i);
  }

  byte* motorSpeedPtr = (byte*)(&motorSpeed);
  for (int i = 0; i < sizeof(int); i++) {
    motorSpeedPtr[i] = EEPROM.read(motorSpeedAddr + i);
  }
}
