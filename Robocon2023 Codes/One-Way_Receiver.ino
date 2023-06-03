#include <esp_now.h>
#include <WiFi.h>


//##################
//#    RECEIVER    #
//##################

typedef struct struct_message {
  double a, b, c;
  int d, e;
} struct_message;

struct_message myData;

//callback function that will be executed when data is received
void receivedData(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Data a: ");
  Serial.println(myData.a);
  Serial.print("Data b: ");
  Serial.println(myData.b);
  Serial.print("Data c: ");
  Serial.println(myData.c);
  Serial.print("Data d: ");
  Serial.println(myData.d);
  Serial.print("Data e: ");
  Serial.println(myData.e);
  Serial.println();
}
 
void setup() {
  Serial.begin(115200);
  
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  esp_now_register_recv_cb(receivedData);
}
 
void loop() {

}
