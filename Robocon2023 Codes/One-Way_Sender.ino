#include "esp_now.h"
#include "WiFi.h"
 
//################
//#    SENDER    #
//################

//    STATUS     //
//5 second LED = the message has been delivered
//blinked 3 times = the messahed is not delivered
//blinked 5 times = error when initializing ESP-NOW
//blinked 8 times = failed to add to peer
//blinked 10 times = sent with success
//blinked 13 times = error sending data

uint8_t broadcastAddress[] = {0xB4, 0x8A, 0x0A, 0x46, 0x9D, 0x70}; //for this I just use chai's esp WIFI ADDRESS

//data that we want to send
typedef struct struct_message {
  double a = 11.11;
  double b = 22.22;
  double c = 33.33;
  int d = 40;
  int e = 50;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void dataStatus(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if(status == ESP_NOW_SEND_SUCCESS){
    digitalWrite(2, HIGH);
    delay(5000);
  }
  else{
    for(int i = 0; i <= 3; i++){
      digitalWrite(2, HIGH);
      delay(1000);
      digitalWrite(2, LOW);
      delay(1000);
    }
  }
}
 
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  pinMode(2, OUTPUT);

  if (esp_now_init() != ESP_OK) {
    for(int i = 1; i <= 5; i++){
      digitalWrite(2, HIGH);
      delay(1000);
      digitalWrite(2, LOW);
      delay(1000);
    }
    return;
  }

  esp_now_register_send_cb(dataStatus);
  
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
         
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    for(int i = 1; i <= 8; i++){
      digitalWrite(2, HIGH);
      delay(1000);
      digitalWrite(2, LOW);
      delay(1000);
    }
    return;
  }
}
 
void loop() {
  myData.a = 555.5;
  myData.a = 333.3;
  myData.a = 666.6;
  myData.a = 12;
  myData.a = 34;
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    for(int i = 1; i <= 10; i++){
      digitalWrite(2, HIGH);
      delay(1000);
      digitalWrite(2, LOW);
      delay(1000);
    }
  }
  else {
    for(int i = 1; i <= 13; i++){
      digitalWrite(2, HIGH);
      delay(1000);
      digitalWrite(2, LOW);
      delay(1000);
    }
  }
  delay(2000);
}
