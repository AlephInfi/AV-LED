#include "..\Visual\v.h"

LEDStrip strip;

void setup(){
  Serial.begin(115200);
  delay(100);
  
  if (!Serial.available()) delay(1000);
  if (Serial.available()) {
    Serial.println("Serial Initialized.");

    strip.Init();
  }
  
  else if (!Serial.available()){ // Flash 3 times red to indicate Serial Error
    strip.Init();
    for(int u = 0; u < 3; u++){
      delay(10);
      for(int i = 0; i < NUM_LEDS; i++){
        strip.setLED(i, RGBColor(255, 0, 0));
      }
      strip.ManualShow();
      delay(500);
      for(int i = 0; i < NUM_LEDS; i++){
        strip.setLED(i, RGBColor(10, 0, 0));
      }
      strip.ManualShow();
      delay(200);
    }
  }
}

void loop(){
  float ratio = (float)(millis() % 5000) / 5000.0f;
  strip.Update(1, ratio);
}