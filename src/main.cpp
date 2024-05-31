#include "..\Visual\v.h"

LEDStrip strip;

void setup(){
  Serial.begin(115200);
  delay(1000);

  strip.Init();
}

void loop(){
  float ratio = (float)(millis() % 5000) / 5000.0f;
  strip.Update(2, ratio);
}