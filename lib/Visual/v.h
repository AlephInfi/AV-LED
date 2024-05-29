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
#define NUM_LEDS        303

#define PULSE_SCALAR    10 //Controls the brightness increase of LEDs when pulsing

class LEDStrip{
    private:
        CRGB led[NUM_LEDS];

        ClockMicros microClock;

        RCA rca;
        BPM bpmDetect;

        RGBColor noiseSpectrum[4] = {RGBColor(0, 255, 0), RGBColor(255, 0, 0), RGBColor(0, 255, 0), RGBColor(0, 0, 255)};
        GradientMaterial gNoiseMat = GradientMaterial(4, noiseSpectrum, 2.0f, false);
        SimplexNoise sNoise = SimplexNoise(1, &gNoiseMat);

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

            float Rmult = Mathematics::Map(Mathematics::Constrain(rca.getBand(1), 0.0f, 120.0f), 0.0f, 120.0f, 0.5f, 16.0f);
            float Gmult = Mathematics::Map(Mathematics::Constrain(rca.getBand(9), 0.0f, 100.0f), 0.0f, 100.0f, 0.5f, 10.0f);
            float Bmult = Mathematics::Map(Mathematics::Constrain(rca.getBand(15), 0.0f, 350.0f), 0.0f, 350.0f, 0.5f, 8.0f);
            float UMult = rca.getGainL(0.2f);

            Serial.print(analogRead(21));
            Serial.print("     ");
            Serial.print(Gmult);
            Serial.print("     ");
            Serial.print(Bmult);
            Serial.print("     ");
            Serial.println(UMult);

            FastLED.setBrightness((uint8_t)Mathematics::Constrain((Rmult / 16.0f * 255.0f), 10.0f, 190.0f));

            for(int num = 0; num < NUM_LEDS; num++){
                CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 0.1f);
                RGBColor refpixel = sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D());
                pixel.r = (uint8_t) (Rmult * refpixel.R * UMult);
                pixel.g = (uint8_t) (Gmult * refpixel.G * UMult);
                pixel.b = (uint8_t) (Bmult * refpixel.B * UMult);
                led[num] = pixel;
            }
        }

        //Use this for pulse amp? uint scalar = Mathematics::Constrain(((uint)Mathematics::Map(rca.Gain, -60.0f, 5.0f, 0.0f, 255.0f)), 0, 255);

        void Pulse(uint8_t amplitude){
            int t = 0;
            int beatGap = (int)(60000.0f / bpmDetect.GetBPM()); // in milliseconds
            CRGB savedLEDState[NUM_LEDS];
            for(int num = 0; num < NUM_LEDS; num++){
                CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()));
                savedLEDState[num] = pixel;
                pixel.addToRGB(amplitude);
                led[num] = pixel;
            }
            while(t < beatGap){
                for(int num = 0; num < NUM_LEDS; num++){
                    CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()));
                    pixel.addToRGB((-1 * amplitude / beatGap));
                    led[num] = pixel;
                }
                t += 1; // millisecond
            }
        }

        void PulseBeat(float ratio){
            float x = 0.5f * 1 / (ratio);

            double BPM = bpmDetect.GetBPM();
        }

    public:
        LEDStrip(){}

        void Init(){
            FastLED.addLeds<WS2812B, LEDPIN, GRB>(led, NUM_LEDS);
            FastLED.setBrightness(120);
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

            switch(command){
                case 0: Simplex(ratio);
                case 1: SimplexWithAudioMod(ratio);
            }

            FastLED.show();
        }
};