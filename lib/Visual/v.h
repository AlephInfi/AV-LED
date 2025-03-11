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

#define MAX_BRIGHTNESS    190 //Maximum brightness

class LEDStrip{
    private:
        CRGB led[NUM_LEDS];

        ClockMicros microClock;

        RCA rca;
        BPM bpmDetect;

        RGBColor noiseSpectrum[3] = {RGBColor(0, 0, 255), RGBColor(255, 0, 0), RGBColor(0, 255, 0)};
        GradientMaterial gNoiseMat = GradientMaterial(3, noiseSpectrum, 2.0f, false);
        SimplexNoise sNoise = SimplexNoise(random16(), &gNoiseMat);
        float currentHue = 0.0f;

        float LowMax = 0.0f;
        float MidMax = 0.0f;
        float HighMax = 0.0f;
        uint8_t currentBrightess = 1;
        int lLow;
        int HighestLowValue = 0;

        unsigned long prevmillis = 0;
        int ct = 0;
        int lgct = 0;
        int MultiplierRandY = random(1, 5);
        int MultiplierRandZ = random(1, 5);

        int RunningAverageDiff;
        uint8_t MinBright = 0;
        int rafilter[50];

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
            float sShift = linSweep * 0.004f + 0.005f;

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
            
            float x = 0.5f * sinf(ratio * 3.14159f / 180.0f * 360.0f * 1 / 5);
            float y = 0.5f * sinf(ratio * 3.14159f / 180.0f * 360.0f * MultiplierRandY / 5 + MultiplierRandY*50);
            float z = 0.5f * sinf(ratio * 3.14159f / 180.0f * 360.0f * MultiplierRandZ / 5 + MultiplierRandZ*100);

            float linSweep = ratio > 0.5f ? 1.0f - ratio : ratio;
            float sShift = linSweep * 0.001f + 0.002f;

            float Rmult = Mathematics::Map(Mathematics::Constrain(lLow, 0.03f, LowMax), 0.01f, LowMax, 0.3f, 10.0f);
            float Gmult = Mathematics::Map(Mathematics::Constrain(rca.getBand(7), 0.03f, MidMax), 0.01f, MidMax, 0.5f, 6.0f);
            float Bmult = Mathematics::Map(Mathematics::Constrain(rca.getBand(15), 0.03f, HighMax), 0.01f, HighMax, 1.0f, 4.0f);
            float UMult = 0.7f;
            
            gNoiseMat.SetGradientPeriod(0.5f + linSweep * 6.0f);
            gNoiseMat.HueShift(currentHue + x * 360);
            sNoise.SetScale(Vector3D(sShift, sShift, sShift));
            sNoise.SetZPosition(x * 4.0f);

            this->currentBrightess = (uint8_t)Mathematics::Constrain((Rmult / 10.0f * MAX_BRIGHTNESS), this->MinBright, MAX_BRIGHTNESS);

            //this->currentBrightess = (uint8_t)(-100.0f/ (currentBrightess - LowMax - 100.0f) - 0.909090909091f);

            if( ((LowMax - lLow) <= this->RunningAverageDiff * 2.55f) && (LowMax >= HighestLowValue / 2) && (rca.getBandAvg() > 120)){
                FastLED.setBrightness((uint8_t) Mathematics::Constrain((float)((currentBrightess + 20)*Rmult/10), this->MinBright, MAX_BRIGHTNESS));

                if (millis() - prevmillis >= 300){
                    gNoiseMat.HueShift(13);
                    currentHue += 13;
                    if (currentHue >= 360.0f){
                        currentHue -= 347.0f;
                    }
                }
                
                int b = random(50, 100);

                for(int num = 0; num < NUM_LEDS; num++){
                    CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 0.1f);
                    pixel.r = MAX_BRIGHTNESS;
                    pixel.g = MAX_BRIGHTNESS - (uint8_t)(b * currentHue * 60 / 3600);
                    pixel.b = MAX_BRIGHTNESS - (uint8_t)(b * currentHue * 60 / 3600);
                    led[num] = pixel;
                }
            }        
            else if( ((LowMax - lLow) > this->RunningAverageDiff * 2.5f) && ((LowMax - lLow) < this->RunningAverageDiff * 3.1f) && (rca.getBandAvg() > 60)){
                FastLED.setBrightness((uint8_t) Mathematics::Constrain(currentBrightess * 0.55f, this->MinBright, MAX_BRIGHTNESS));

                for(int num = 0; num < NUM_LEDS; num++){
                    CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 0.1f);
                    RGBColor refpixel = sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D());
                    pixel.r = (uint8_t) Mathematics::Constrain((Rmult * refpixel.R * UMult), 0, 255);
                    pixel.g = (uint8_t) Mathematics::Constrain((Gmult * refpixel.G * UMult), 0, 255);
                    pixel.b = (uint8_t) Mathematics::Constrain((Bmult * refpixel.B * UMult), 0, 255);
                    led[num] = pixel;
                }
            }
            else{
                FastLED.setBrightness((uint8_t)(Mathematics::Constrain(currentBrightess * 0.4f, this->MinBright, MAX_BRIGHTNESS)));
                for(int num = 0; num < NUM_LEDS; num++){
                    CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 0.1f);
                    RGBColor refpixel = sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D());
                    if (lLow > 2){
                        pixel.r = (uint8_t) Mathematics::Constrain((Mathematics::Map(Mathematics::Constrain(RunningAverageDiff, 0.03f, LowMax), 0.01f, LowMax, 0.3f, 10.0f) * refpixel.R * UMult), this->MinBright, 255);
                    }
                    else pixel.r = (uint8_t) Mathematics::Constrain((x*UMult*refpixel.R), this->MinBright, 255);
                    if (rca.getBand(7) > 2){
                        pixel.g = (uint8_t) Mathematics::Constrain((Gmult * (refpixel.G) * UMult), this->MinBright, 255);
                    }
                    else pixel.g = (uint8_t) Mathematics::Constrain((y*UMult*refpixel.G), this->MinBright, 255);
                    if (rca.getBand(15) > 15){
                        pixel.b = (uint8_t) Mathematics::Constrain((Bmult * (refpixel.B) * UMult), this->MinBright, 255);
                    }
                    else pixel.b = (uint8_t) Mathematics::Constrain((z*UMult*refpixel.B), this->MinBright, 255);
                    led[num] = pixel;
                }
            }
            
/*
            Serial.print(Rmult);
            Serial.print(" rmult \t\t");
            
            
            Serial.print(" raw band out \t\t");
            Serial.print(this->RunningAverageDiff);
            Serial.print(" RAFilter \t\t");
            Serial.print((LowMax - lLow));
            Serial.print(" Difference \t\t");
            Serial.print(FastLED.getBrightness());
            Serial.print(" FPS \t\t");
            
            Serial.println(FastLED.getFPS());
            */
        }
/*
        void AudioPulse(float ratio){
            float x = 0.5f * sinf(ratio * 3.14159f / 180.0f * 360.0f * 2.0f);

            float linSweep = ratio > 0.5f ? 1.0f - ratio : ratio;
            float sShift = linSweep * 0.002f + 0.005f;

            gNoiseMat.SetGradientPeriod(0.5f + linSweep * 6.0f);
            gNoiseMat.HueShift(ratio * 360 * 2);
            sNoise.SetScale(Vector3D(sShift, sShift, sShift));
            sNoise.SetZPosition(x * 4.0f);

            float Rmult = Mathematics::Map(Mathematics::Constrain(lLow, 0.03f, LowMax), 0.01f, LowMax, 1.0f, 10.0f);
            float Gmult = Mathematics::Map(Mathematics::Constrain(rca.getBand(7), 0.03f, MidMax), 0.01f, MidMax, 1.0f, 6.0f);
            float Bmult = Mathematics::Map(Mathematics::Constrain(rca.getBand(15), 0.03f, HighMax), 0.01f, HighMax, 1.0f, 4.0f);

            

            FastLED.setBrightness((uint8_t)Mathematics::Map((Rmult * 0.1f * MAX_BRIGHTNESS), (0.1f * MAX_BRIGHTNESS), MAX_BRIGHTNESS, 3, MAX_BRIGHTNESS));

            for(int num = 0; num < NUM_LEDS; num++){
                CRGB pixel;
                RGBColor refpixel = sNoise.GetRGB(Vector3D(num / 5.0f,num / 2.0f,num / 3.0f), Vector3D(), Vector3D());
                if (lLow >= rca.getBand(7) && lLow >= rca.getBand(15)){
                    pixel.r = (uint8_t) (refpixel.R * Rmult);
                    pixel.g = (uint8_t) (refpixel.G / (8.0f * Gmult));
                    pixel.b = (uint8_t) (refpixel.B / (8.0f * Bmult));
                }
                if (rca.getBand(7) >= lLow && rca.getBand(7) >= rca.getBand(15)){
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
*/
    public:
        LEDStrip(){}

        void Init(uint8_t min){
            FastLED.addLeds<WS2812B, LEDPIN, GRB>(led, NUM_LEDS);
            FastLED.setBrightness(min);

            this->MinBright = min;

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
            //lLow = (rca.getBand(0) > rca.getBand(1)) ? rca.getBand(0) : rca.getBand(1);
            lLow = rca.getBand(0);
            if (ct >= 50) ct = 0;

            if(lLow > LowMax) LowMax = lLow;
            if(rca.getBand(7) > MidMax) MidMax = rca.getBand(7);
            if(rca.getBand(15) > HighMax) HighMax = rca.getBand(15);

            if(millis() - prevmillis >= 5000){
                LowMax = 0.1f;
                if(rca.getBand(7) > 1) MidMax = 0.1f;
                if(rca.getBand(15) > 1) HighMax = 0.1f;
                prevmillis = millis();

                if (lgct >= 10){
                    HighestLowValue = LowMax;
                    lgct = 0;
                } 
                lgct++;
            }

            if(lLow > LowMax/2) rafilter[ct] = LowMax - lLow;
            ct++;

            int avg = 0;
            for(int a = 0; a < 50; a++){
                avg += rafilter[a];
            }
            this->RunningAverageDiff = (int)(avg / 50);

            switch(command){
                case 0: Simplex(ratio);
                case 1: SimplexWithAudioMod(ratio);
                //case 2: AudioPulse(ratio);
            }

            FastLED.show();
        }
};