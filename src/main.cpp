#include "v.h"
#include "Bluetooth.h"

Bluetooth BTDevice;
LEDStrip strip;

void setup(){
  Serial.begin(115200);
  BTDevice.BTBegin();
  strip.Init();
}

void loop(){
  float ratio = (float)(millis() % 5000) / 5000.0f;
  BTDevice.Update();
  Serial.print("Left Reading:  ");
  Serial.print(analogReadRaw(32));
  Serial.print("  Right Reading:  ");
  Serial.println(analogReadRaw(33));
}