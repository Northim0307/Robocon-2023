#include <EEPROM.h>

//if the float data is higher, the next variable must skip 1 address
//if the int data is higher, the next variable must skip 2 address

float inita = 10.66;
int initb = 50;
int initc = 70;
int initd = 80;
int inite = 90;
int initf = 100;

float a = 0.00;
int b = 0.00;
int c = 0;
int d = 0;
int e = 0;
int f = 0;

void saveData(float aa, int bb, int cc, int dd, int ee, int ff) {
  EEPROM.writeFloat(0, aa);
  EEPROM.writeInt(5, bb);
  EEPROM.writeInt(10, cc);
  EEPROM.writeInt(14, dd);
  EEPROM.writeInt(20, ee);
  EEPROM.writeInt(25, ff);
  EEPROM.commit();
}

void readData() {
  a = EEPROM.readFloat(0);
  b = EEPROM.readInt(5);
  c = EEPROM.readInt(10);
  d = EEPROM.readInt(14);
  e = EEPROM.readInt(20);
  f = EEPROM.readInt(25);
}

void updateData(float aa, int bb, int cc, int dd, int ee, int ff) {
  EEPROM.writeFloat(0, aa);
  EEPROM.writeInt(5, bb);
  EEPROM.writeInt(10, cc);
  EEPROM.writeInt(14, dd);
  EEPROM.writeInt(20, ee);
  EEPROM.writeInt(25, ff);
  EEPROM.commit();
}

void printData(){
  Serial.print("Saved A: ");
  Serial.println(a);
  Serial.print("Saved B: ");
  Serial.println(b);
  Serial.print("Saved C: ");
  Serial.println(c);
  Serial.print("Saved D: ");
  Serial.println(d);
  Serial.print("Saved E: ");
  Serial.println(e);
  Serial.print("Saved F: ");
  Serial.println(f);
  Serial.println("###########\n");
}

void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);

  saveData(inita, initb, initc, initd, inite, initf);
  readData();
  printData();

  updateData(100.66, 200, 300, 400, 500, 600);
  readData();
  printData();
}

void loop() {
  
}
