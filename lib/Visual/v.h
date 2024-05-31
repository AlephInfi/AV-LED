#pragma once

#include <FastLED.h>
#include "..\Audio\BPMDetect.h"

#include "..\Materials\SimplexNoise.h"

/* for 4-pin LEDs
#define LEDCLK          14
#define LEDDAT          2
*/
#define LED_VOLTS       5
#define LEDPIN          14
#define NUM_LEDS        600

#define MAX_BRIGHTNESS    190 //Miximum brightness

class LEDStrip{
    private:
        CRGB led[NUM_LEDS];

        ClockMicros microClock;

        RCA rca;
        BPM bpmDetect;

        RGBColor noiseSpectrum[4] = {RGBColor(0, 255, 0), RGBColor(255, 0, 0), RGBColor(0, 255, 0), RGBColor(0, 0, 255)};
        GradientMaterial gNoiseMat = GradientMaterial(4, noiseSpectrum, 2.0f, false);
        SimplexNoise sNoise = SimplexNoise(1, &gNoiseMat);

        float LowMax = 0.0f;
        float MidMax = 0.0f;
        float HighMax = 0.0f;

        unsigned long prevmillis = 0;

        // do cool bpm effect with color switching
        SimplexNoise GenRandTwoColorSimplex(){
            RGBColor noiseSpectrum[2] = {RGBColor(255 - random8(120), 255 - random8(120), 255 - random8(120)), RGBColor(255 - random8(120), 255 - random8(120), 255 - random8(120))};
            GradientMaterial gNoiseMat = GradientMaterial(4, noiseSpectrum, 2.0f, false);
            SimplexNoise sNoise = SimplexNoise(1, &gNoiseMat);
            return sNoise;
        }

        void Simplex(float ratio){
            float x = 0.5f * sinf(ratio * 3.14159f / 180.0f * 360.0f * 2.0f);

            float linSweep = ratio > 0.5f ? 1.0f - ratio : ratio;
            float sShift = linSweep * 0.002f + 0.005f;

            gNoiseMat.SetGradientPeriod(0.5f + linSweep * 6.0f);
            gNoiseMat.HueShift(ratio * 360 * 2);
            sNoise.SetScale(Vector3D(sShift, sShift, sShift));
            sNoise.SetZPosition(x * 4.0f);

            for(int num = 0; num < NUM_LEDS; num++){
                CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()));
                led[num] = pixel;
            }
        }

        void SimplexWithAudioMod(float ratio){
            float x = 0.5f * sinf(ratio * 3.14159f / 180.0f * 360.0f * 2.0f);
            //float y = 0.5f * cosf(ratio * 3.14159f / 180.0f * 360.0f * 3.0f);

            float linSweep = ratio > 0.5f ? 1.0f - ratio : ratio;
            float sShift = linSweep * 0.002f + 0.005f;

            gNoiseMat.SetGradientPeriod(0.5f + linSweep * 6.0f);
            gNoiseMat.HueShift(ratio * 360 * 2);
            sNoise.SetScale(Vector3D(sShift, sShift, sShift));
            sNoise.SetZPosition(x * 4.0f);

            float Rmult = Mathematics::Map(Mathematics::Constrain(rca.getBand(0), 0.03f, LowMax), 0.01f, LowMax, 1.0f, 10.0f);
            float Gmult = Mathematics::Map(Mathematics::Constrain(rca.getBand(7), 0.03f, MidMax), 0.01f, MidMax, 1.0f, 6.0f);
            float Bmult = Mathematics::Map(Mathematics::Constrain(rca.getBand(15), 0.03f, HighMax), 0.01f, HighMax, 1.0f, 4.0f);
            float UMult = rca.getGainL(0.2f);

            FastLED.setBrightness((uint8_t)Mathematics::Constrain((Rmult / 16.0f * 255.0f), 1.0f, 200.0f));

            for(int num = 0; num < NUM_LEDS; num++){
                CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 0.1f);
                RGBColor refpixel = sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D());
                pixel.r = (uint8_t) (Rmult * refpixel.R * UMult);
                pixel.g = (uint8_t) (Gmult * refpixel.G * UMult);
                pixel.b = (uint8_t) (Bmult * refpixel.B * UMult);
                led[num] = pixel;
            }
        }

        void AudioPulse(float ratio){
            float x = 0.5f * sinf(ratio * 3.14159f / 180.0f * 360.0f * 2.0f);

            float linSweep = ratio > 0.5f ? 1.0f - ratio : ratio;
            float sShift = linSweep * 0.002f + 0.005f;

            gNoiseMat.SetGradientPeriod(0.5f + linSweep * 6.0f);
            gNoiseMat.HueShift(ratio * 360 * 2);
            sNoise.SetScale(Vector3D(sShift, sShift, sShift));
            sNoise.SetZPosition(x * 4.0f);

            float Rmult = Mathematics::Map(Mathematics::Constrain(rca.getBand(0), 0.03f, LowMax), 0.01f, LowMax, 1.0f, 10.0f);
            float Gmult = Mathematics::Map(Mathematics::Constrain(rca.getBand(7), 0.03f, MidMax), 0.01f, MidMax, 1.0f, 6.0f);
            float Bmult = Mathematics::Map(Mathematics::Constrain(rca.getBand(15), 0.03f, HighMax), 0.01f, HighMax, 1.0f, 4.0f);

            Serial.print(Rmult);
            Serial.print("\t");
            Serial.print(Gmult);
            Serial.print("\t");
            Serial.print(Bmult);
            Serial.print("\t");
            Serial.print(FastLED.getBrightness());
            Serial.print("\t");
            Serial.println(FastLED.getFPS());

            FastLED.setBrightness((uint8_t)Mathematics::Map((Rmult * 0.1f * MAX_BRIGHTNESS), (0.1f * MAX_BRIGHTNESS), MAX_BRIGHTNESS, 3, MAX_BRIGHTNESS));

            for(int num = 0; num < NUM_LEDS; num++){
                CRGB pixel;
                RGBColor refpixel = sNoise.GetRGB(Vector3D(num / 5.0f,num / 2.0f,num / 3.0f), Vector3D(), Vector3D());
                if (rca.getBand(0) >= rca.getBand(7) && rca.getBand(0) >= rca.getBand(15)){
                    pixel.r = (uint8_t) (refpixel.R * Rmult);
                    pixel.g = (uint8_t) (refpixel.G / (8.0f * Gmult));
                    pixel.b = (uint8_t) (refpixel.B / (8.0f * Bmult));
                }
                if (rca.getBand(7) >= rca.getBand(0) && rca.getBand(7) >= rca.getBand(15)){
                    pixel.r = (uint8_t) (refpixel.R / Rmult);
                    pixel.g = (uint8_t) (refpixel.G * 0.25f * Gmult);
                    pixel.b = (uint8_t) (refpixel.B / (4.0f * Bmult));
                }
                else{
                    pixel.r = (uint8_t) (refpixel.R / Rmult);
                    pixel.g = (uint8_t) (refpixel.G / (4.0f * Gmult));
                    pixel.b = (uint8_t) (refpixel.B * 0.25f * Bmult);
                }
                
                led[num] = pixel;
            }
        }

    public:
        LEDStrip(){}

        void Init(){
            FastLED.addLeds<WS2812B, LEDPIN, GRB>(led, NUM_LEDS);
            FastLED.setBrightness(0);

            bpmDetect.SetRange();
        }

        void setLED(uint16_t pixel, RGBColor color){
            led[pixel].r = color.R;
            led[pixel].g = color.G;
            led[pixel].b = color.B;
        }
        void ManualShow(){
            FastLED.show();
        }

        double getBand(uint8_t band){
            return rca.getBand(band);
        }

        void Update(int command, float ratio){
            rca.Sample();
            rca.FFT();
            //bpmDetect.Update(rca); // Mem overflow?

            if(rca.getBand(0) > LowMax) LowMax = rca.getBand(0);
            if(rca.getBand(7) > MidMax) MidMax = rca.getBand(7);
            if(rca.getBand(15) > HighMax) HighMax = rca.getBand(15);

            if(millis() - prevmillis >= 5000){
                if(rca.getBand(0) > 1) LowMax = 0.1f;
                if(rca.getBand(7) > 1) MidMax = 0.1f;
                if(rca.getBand(15) > 1) HighMax = 0.1f;
                prevmillis = millis();
            }

            switch(command){
                case 0: Simplex(ratio);
                case 1: SimplexWithAudioMod(ratio);
                case 2: AudioPulse(ratio);
            }

            FastLED.show();
        }
};