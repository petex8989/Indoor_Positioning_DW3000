#include "WiFi.h"
 
void setup(){
  Serial.begin(38400);
  WiFi.mode(WIFI_MODE_STA);
  Serial.println(WiFi.macAddress());
}
 
void loop(){

}