#include "..\Visual\v.h"

LEDStrip strip;

void setup(){
  #ifdef DEBUG
  Serial.begin(115200);
  while (!Serial) {;}
  #endif

  strip.Init(27);
}

void loop(){
  float ratio = (float)(millis() % 15000) / 15000.0f;
  strip.Update(1, ratio);
}