#include <Arduino.h>
#include <FastLED.h>

class LEDStrip{
    private:
        static const int len = 1;
        CRGB led[len];

    public:
        LEDStrip(int ledCt){}
        static void Init(CRGB leds[], int n){
            FastLED.addLeds<WS2811,2,RGB>(leds, n);
            FastLED.setBrightness(150);
        }
};