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
        CRGB prevLed[NUM_LEDS];

        ClockMicros microClock;

        RCA rca;
        BeatDetector bpm;

        RGBColor noiseSpectrum[3] = {RGBColor(0, 0, 255), RGBColor(255, 0, 0), RGBColor(0, 255, 0)};
        GradientMaterial gNoiseMat = GradientMaterial(3, noiseSpectrum, 2.0f, false);
        SimplexNoise sNoise = SimplexNoise(random16(), &gNoiseMat);
        float currentHue = 0.0f;

        float LowMax = 0.0f;
        float MidMax = 0.0f;
        float HighMax = 0.0f;
        uint8_t currentBrightess = 1;
        uint8_t prevBrightness = 0;
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

        SimplexNoise GenRandTwoColorSimplex(){
            RGBColor noiseSpectrum[2] = {RGBColor(255 - random8(120), 255 - random8(120), 255 - random8(120)), RGBColor(255 - random8(120), 255 - random8(120), 255 - random8(120))};
            GradientMaterial gNoiseMat = GradientMaterial(4, noiseSpectrum, 2.0f, false);
            SimplexNoise sNoise = SimplexNoise(1, &gNoiseMat);
            return sNoise;
        }

        float GrayScale(float R, float G, float B, float saturation){
            return saturation * (R + G + B)/3 - currentBrightess*60/255;
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

        void SimplexWithAudioMod(float ratio, bool IdleState){

            auto smoothing = [&] (int num, CRGB pixel)
            {
                // Smoothing factor between 0.0 and 1.0  
                // Lower = smoother (slower changes), Higher = faster response
                const float smoothFactor = 0.8f;  

                CRGB oldPix = prevLed[num];

                // Compute smoothed RGB using linear interpolation
                pixel.r = oldPix.r + (pixel.r - oldPix.r) * smoothFactor;
                pixel.g = oldPix.g + (pixel.g - oldPix.g) * smoothFactor;
                pixel.b = oldPix.b + (pixel.b - oldPix.b) * smoothFactor;

                // Write smoothed pixel to LED strip
                led[num] = pixel;

                // Store current value for next frame
                prevLed[num] = pixel;
            };

            if (IdleState) {
            float x = 0.5f * sinf(ratio * 2 * 3.14159f) + 0.5f;

            float linSweep = ratio > 0.5f ? 1.0f - ratio : ratio;
            float sShift = linSweep * 0.00075f + 0.0015f;
            
            gNoiseMat.SetGradientPeriod(0.5f + linSweep * 6.0f);
            gNoiseMat.HueShift(currentHue + x * 360);
            sNoise.SetScale(Vector3D(sShift, sShift, sShift));
            sNoise.SetZPosition(x * 8.0f);
            const float idleBrightSmooth = 0.08f;
            uint8_t smoothed = prevBrightness + (MinBright - prevBrightness) * idleBrightSmooth;
            FastLED.setBrightness(smoothed);
            prevBrightness = smoothed;

            for(int num = 0; num < NUM_LEDS; num++){
                CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 0.1f);
                pixel.nscale8_video(65); // keep soft & not eye-searing
                led[num] = pixel;
                prevLed[num] = pixel;
            }
            return;
            }
            
            float x = 0.5f * sinf(ratio * 2 * 3.14159f) + 0.5f;
            float y = 0.5f * sinf(ratio * 4 * 3.14159f + MultiplierRandY*50) + 0.5f;
            float z = 0.5f * sinf(ratio * 2 * 3.14159f + MultiplierRandZ*100) + 0.5f;

            float linSweep = ratio > 0.5f ? 1.0f - ratio : ratio;
            float sShift = linSweep * 0.00075f + 0.0015f;

            float Rmult = Mathematics::Map(Mathematics::Constrain(lLow, 0.03f, LowMax), 0.01f, LowMax, 0.3f, 10.0f);
            float Gmult = Mathematics::Map(Mathematics::Constrain(rca.getBandAvg(4, 10) - 1, 0.03f, MidMax), 0.01f, MidMax, 0.5f, 6.0f);
            float Bmult = Mathematics::Map(Mathematics::Constrain(rca.getBandAvg(10, 15), 0.03f, HighMax), 0.01f, HighMax, 1.0f, 4.0f);
            float UMult = 0.7f;
            
            gNoiseMat.SetGradientPeriod(0.5f + linSweep * 6.0f);
            gNoiseMat.HueShift(currentHue + x * 360);
            sNoise.SetScale(Vector3D(sShift, sShift, sShift));
            sNoise.SetZPosition(x * 8.0f);

            if (rca.getBandAvg(0, 3) > 3) this->currentBrightess = (uint8_t)Mathematics::Constrain((Rmult / 10.0f * MAX_BRIGHTNESS), this->MinBright, MAX_BRIGHTNESS);

            //this->currentBrightess = (uint8_t)(-100.0f/ (currentBrightess - LowMax - 100.0f) - 0.909090909091f);

            if( ((LowMax - lLow) <= this->RunningAverageDiff * 2.55f) && (LowMax >= HighestLowValue / 2) && (rca.getBandAvg(0, 3) > 10)   && (rca.getBandAvg(0, 3) > LowMax*0.65f)){
                FastLED.setBrightness((uint8_t) Mathematics::Constrain((float)((currentBrightess + 60)*Rmult/10), this->MinBright, MAX_BRIGHTNESS));

                if (millis() - prevmillis >= 600){
                    gNoiseMat.HueShift(7);
                    currentHue += 7;
                    if (currentHue >= 360.0f){
                        currentHue -= 353.0f;
                    }
                }
                
                int b = random(50, 100);

                for(int num = 0; num < NUM_LEDS; num++){
                    CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 0.1f);
                    pixel.r = MAX_BRIGHTNESS;
                    pixel.g = MAX_BRIGHTNESS - (uint8_t)(b * currentHue / 100);
                    pixel.b = MAX_BRIGHTNESS - (uint8_t)(b * currentHue / 100);
                    smoothing(num, pixel);
                }
            }        
            else if( ((LowMax - lLow) > this->RunningAverageDiff * 2.5f) && ((LowMax - lLow) < this->RunningAverageDiff * 3.1f) && (rca.getBandAvg(0, 3) > 10)  && (rca.getBandAvg(0, 3) <= LowMax*0.65f) ){
                FastLED.setBrightness((uint8_t) Mathematics::Constrain(MinBright + currentBrightess*Rmult/11, this->MinBright, MAX_BRIGHTNESS));

                for(int num = 0; num < NUM_LEDS; num++){
                    CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 0.1f);
                    RGBColor refpixel = sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D());
                    pixel.r = (uint8_t) Mathematics::Constrain((Rmult * refpixel.R * UMult), 0, 255);
                    pixel.g = (uint8_t) Mathematics::Constrain((Gmult * refpixel.G * UMult), 0, 255);
                    pixel.b = (uint8_t) Mathematics::Constrain((Bmult * refpixel.B * UMult), 0, 255);
                    smoothing(num, pixel);
                }
            }
            else{
                if (rca.getBandAvg(10,16) > 20 || rca.getBandAvg(0,3) > 5 || rca.getBandAvg(3,10) > 2) {
                    FastLED.setBrightness((uint8_t)(Mathematics::Constrain(MinBright + currentBrightess*Rmult/12, this->MinBright, MAX_BRIGHTNESS)));
                }

                if (FastLED.getBrightness() > MinBright) FastLED.setBrightness(FastLED.getBrightness()-1);

                int sat = 0.8f;

                for(int num = 0; num < NUM_LEDS; num++){
                    CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 0.1f);
                    RGBColor refpixel = sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D());
                    float gray = GrayScale(pixel.r, pixel.g, pixel.b, sat);

                    if (rca.getBandAvg(0, 3) > 10){
                        pixel.r = (uint8_t) Mathematics::Constrain(Rmult * refpixel.R * UMult, 60, 255);
                    }
                    else pixel.r = (uint8_t) Mathematics::Constrain((x*UMult*refpixel.R)*(1-sat) + gray, 60, 255);
                    if (rca.getBandAvg(4, 10) > 10){
                        pixel.g = (uint8_t) Mathematics::Constrain((Gmult * (refpixel.G) * UMult), 50, 255);
                    }
                    else pixel.g = (uint8_t) Mathematics::Constrain((y*UMult*refpixel.G)*(1-sat) + gray, 50, 255);
                    if (rca.getBandAvg(10, 16) > 15){
                        pixel.b = (uint8_t) Mathematics::Constrain((Bmult * (refpixel.B) * UMult), 60, 255);
                    }
                    else pixel.b = (uint8_t) Mathematics::Constrain((z*UMult*refpixel.B)*(1-sat) + gray, 60, 255);
                    smoothing(num, pixel);
                }

            }
        }

        void AudioPulse(float audioLevel) {
            // ----- STATE (persists automatically) -----
            struct Pulse {
                float pos;
                float speed;
                float width;
                CHSV color;
                bool active;
            };
            static const uint8_t MAX_PULSES = 8;
            static Pulse pulses[MAX_PULSES];

            // Beat envelope state
            static float envelope = 0.0f;
            static bool beatHeld = false;

            // Tunable beat detector sensitivity
            const float attack = 0.70f;
            const float release = 0.12f;
            const float sensitivity = 1.45f;

            // ----- BEAT DETECTION -----
            if (audioLevel > envelope)
                envelope += attack * (audioLevel - envelope);
            else
                envelope += release * (audioLevel - envelope);

            float threshold = envelope * sensitivity;
            bool beatNow = (audioLevel > threshold);
            bool beat = (beatNow && !beatHeld);
            beatHeld = beatNow;

            // ----- SPAWN PULSE ON BEAT (Left → Right) -----
            if (beat) {
                for (uint8_t i = 0; i < MAX_PULSES; i++) {
                    if (!pulses[i].active) {
                        pulses[i].pos = 0.0f;        // Start at left (pixel 0)
                        pulses[i].speed = 0.65f;     // Try 0.45 - 1.2 for different feels
                        pulses[i].width = 6.0f;      // Spread / bloom size
                        pulses[i].color = CHSV(random8(), 255, 255);
                        pulses[i].active = true;
                        break;
                    }
                }
            }

            // ----- CLEAR STRIP (no background; pulses only) -----
            for (int i = 0; i < NUM_LEDS; i++)
                led[i] = CRGB::Black;

            // ----- UPDATE + DRAW PULSES -----
            for (uint8_t i = 0; i < MAX_PULSES; i++) {
                if (!pulses[i].active) continue;

                pulses[i].pos += pulses[i].speed;

                // If pulse has moved off strip → remove
                if (pulses[i].pos - pulses[i].width > (float)NUM_LEDS) {
                    pulses[i].active = false;
                    continue;
                }

                // Paint pulse area
                for (int px = 0; px < NUM_LEDS; px++) {
                    float d = fabsf(px - pulses[i].pos) / pulses[i].width;
                    if (d > 1.0f) continue;

                    float intensity = 1.0f - d;  // linear falloff
                    led[px] += pulses[i].color % (uint8_t)(intensity * 255);
                }
            }

            FastLED.show();
        }

    public:
        LEDStrip(){}

        void Init(uint8_t min){
            FastLED.addLeds<WS2812B, LEDPIN, GRB>(led, NUM_LEDS);
            FastLED.setBrightness(min);

            this->MinBright = min;
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
            auto smoothBright = [&] (){
                // --- PUNCHY BRIGHTNESS ENVELOPE (Club Mode) --- //
                float bass = Mathematics::Constrain(rca.getBandAvg(0,3), 0.0f, LowMax);

                // Normalize bass energy to 0–1
                float bassNorm = bass / (LowMax + 0.0001f);
                bassNorm = powf(bassNorm, 0.55f);   // nonlinear → stronger accent on peaks

                // Map to desired brightness range
                uint8_t targetBrightness = MinBright + bassNorm * (MAX_BRIGHTNESS - MinBright);

                // Fast attack, slower release
                float attack = 0.85f;   // faster = more punch
                float release = 0.18f;  // slower fade = breathing effect

                float smoothing = (targetBrightness > prevBrightness) ? attack : release;

                uint8_t smoothedBrightness = prevBrightness + (targetBrightness - prevBrightness) * smoothing;

                FastLED.setBrightness(smoothedBrightness);
                prevBrightness = smoothedBrightness;
            };

            rca.Sample();
            rca.FFT();

            float low  = rca.getBandAvg(0,3);
            float mid  = rca.getBandAvg(4,10);
            float high = rca.getBandAvg(10,16);

            static bool noAudio = false;
            static uint32_t silenceTimer = 0;

            // If all bands are consistently below noise floor → silence
            if (low < 5.0f && mid < 5.0f && high < 20.0f) {
                if (millis() - silenceTimer > 120) noAudio = true;
            } else {
                silenceTimer = millis();
                noAudio = false;
            }

            lLow = rca.getBandAvg(0,3);
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
                case 1: SimplexWithAudioMod(ratio, noAudio);
                //case 2: AudioPulse(rca.getBandAvg(0, 3));
            }


            smoothBright();
            FastLED.show();
        }
};