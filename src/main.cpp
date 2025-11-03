#include "..\Visual\v.h"

LEDStrip strip;

void setup(){
  Serial.begin(115200);
  delay(1000);

  strip.Init(20);
}

void loop(){
  float ratio = (float)(millis() % 10000) / 10000.0f;
  strip.Update(1, ratio);
}