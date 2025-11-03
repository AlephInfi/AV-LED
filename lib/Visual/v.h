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

#define MAX_BRIGHTNESS    160 //Maximum brightness

class LEDStrip{
    private:
        CRGB led[NUM_LEDS];
        CRGB prevLed[NUM_LEDS];

        ClockMicros microClock;

        RCA rca;
        BeatDetector bpm;

        RGBColor noiseSpectrum[3] = {RGBColor(0, 0, 255), RGBColor(255, 0, 0), RGBColor(0, 255, 0)};
        //RGBColor noiseSpectrum[3] = {RGBColor(0, 0, 0), RGBColor(0, 0, 0), RGBColor(0, 0, 0)};
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
        
        CRGB trans_white_pixel = CRGB(150, 150, 150);

        SimplexNoise GenRandTwoColorSimplex(){
            RGBColor noiseSpectrum[2] = {RGBColor(255 - random8(120), 255 - random8(120), 255 - random8(120)), RGBColor(255 - random8(120), 255 - random8(120), 255 - random8(120))};
            //RGBColor noiseSpectrum[2] = {RGBColor(0, 0, 0), RGBColor(0, 0, 0)};
            GradientMaterial gNoiseMat = GradientMaterial(4, noiseSpectrum, 2.0f, false);
            SimplexNoise sNoise = SimplexNoise(1, &gNoiseMat);
            return sNoise;
        }

        float GrayScale(float R, float G, float B, float saturation){
            return saturation * (R + G + B)/3 - currentBrightess*60/255;
        }

        struct Pulse {
            CRGB color;
            float length;   // tail length in LEDs
            uint32_t startTime;
            uint32_t duration;   // ms to travel full strip  
        };

        static const uint8_t MAX_PULSES = 20;
        uint32_t pulseTimeoutDuration;
        Pulse pulses[MAX_PULSES];
        uint8_t pulseCount = 0;

        void RenderPulses() {
            uint32_t now = millis();

            for (uint8_t i = 0; i < pulseCount; i++) {
                Pulse &p = pulses[i];

                // progress from 0.0 → 1.0
                float progress = (float)(now - p.startTime) / (float)p.duration;

                if (progress >= 1.0f) {
                    // Pulse finished → remove it
                    for (uint8_t j = i; j < pulseCount - 1; j++) pulses[j] = pulses[j+1];
                    pulseCount--;
                    i--;
                    continue;
                }

                float headPos = progress * (NUM_LEDS - 1);

                // Draw from head backward toward tail
                // Draw from head back to tail
                for (int ledIndex = (int)headPos; ledIndex >= (int)(headPos - p.length) && ledIndex >= 0; ledIndex--) {

                    float d = (headPos - ledIndex) / p.length;
                    d = Mathematics::Constrain(d, 0.0f, 1.0f);

                    // Exponential shaping
                    const float headSharpness = 4.0f; // how hot the head is
                    const float tailFalloff   = 3.5f; // how fast tail fades

                    float intensity = expf(-tailFalloff * d);
                    float colorBoost = expf(-headSharpness * d);

                    intensity *= (1.0f + 0.5f * colorBoost);
                    intensity = Mathematics::Constrain(intensity, 0.0f, 1.0f);

                    CRGB faded = CRGB(
                        p.color.r * intensity,
                        p.color.g * intensity,
                        p.color.b * intensity
                    );

                    led[ledIndex].r = Mathematics::Constrain((unsigned int)led[ledIndex].r + faded.r, 0, 255);
                    led[ledIndex].g = Mathematics::Constrain((unsigned int)led[ledIndex].g + faded.g, 0, 255);
                    led[ledIndex].b = Mathematics::Constrain((unsigned int)led[ledIndex].b + faded.b, 0, 255);
                }

            }
        }

        void StartPulse(CRGB color, float length, uint32_t t_ms, uint16_t timeSince = 300) {
            if (pulseCount >= MAX_PULSES) return; // no room, ignore

            pulseTimeoutDuration = (length / NUM_LEDS) * t_ms;
            pulses[pulseCount].color = color;
            pulses[pulseCount].length = Mathematics::Constrain(length * timeSince / 500, 30.0f, 70.0f);
            pulses[pulseCount].startTime = millis();
            pulses[pulseCount].duration = Mathematics::Constrain(t_ms * timeSince / 500, 1300.0f, 4000.0f);
            pulseCount++;
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
                const float smoothFactor = 0.6f;  

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
            
            static unsigned long prevT = millis();

            float Rmult = Mathematics::Map(Mathematics::Constrain(lLow, 0.03f, LowMax), 0.01f, LowMax, 0.3f, 10.0f);
            float Gmult = Mathematics::Map(Mathematics::Constrain(rca.getBandAvg(4, 10) - 1, 0.03f, MidMax), 0.01f, MidMax, 0.5f, 6.0f);
            float Bmult = Mathematics::Map(Mathematics::Constrain(rca.getBandAvg(10, 15), 0.03f, HighMax), 0.01f, HighMax, 1.0f, 4.0f);
            float UMult = 0.7f;

            if (IdleState) {
                float x = 0.5f * sinf(ratio * 2 * 3.14159f) + 0.5f;
                float y = 0.5f * sinf(ratio * 4 * 3.14159f + MultiplierRandY*50) + 0.5f;
                float z = 0.5f * sinf(ratio * 2 * 3.14159f + MultiplierRandZ*100) + 0.5f;

                float linSweep = ratio > 0.5f ? 1.0f - ratio : ratio;
                float sShift = linSweep * 0.001f;
                
                gNoiseMat.SetGradientPeriod(0.5f + linSweep * 6.0f);
                gNoiseMat.HueShift(currentHue + x * 180);
                sNoise.SetScale(Vector3D(sShift, sShift, sShift));
                sNoise.SetZPosition(x * 8.0f);
                FastLED.setBrightness(MinBright);

                int sat = 0.8f;

                for(int num = 0; num < NUM_LEDS; num++){
                    CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 1.0f);
                        RGBColor refpixel = sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D());
                        float gray = GrayScale(pixel.r, pixel.g, pixel.b, sat);
                        pixel.r = (uint8_t) Mathematics::Constrain((x*UMult*refpixel.R)*(1-sat) + gray, 60, 255);
                        pixel.g = (uint8_t) Mathematics::Constrain((y*UMult*refpixel.G)*(1-sat) + gray, 50, 255);
                        pixel.b = (uint8_t) Mathematics::Constrain((z*UMult*refpixel.B)*(1-sat) + gray, 60, 255);
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
            
            gNoiseMat.SetGradientPeriod(0.5f + linSweep * 6.0f);
            gNoiseMat.HueShift(currentHue + x * 360);
            sNoise.SetScale(Vector3D(sShift, sShift, sShift));
            sNoise.SetZPosition(x * 8.0f);

            if (rca.getBandAvg(0, 3) > 3) this->currentBrightess = (uint8_t)Mathematics::Constrain((Rmult / 10.0f * MAX_BRIGHTNESS), this->MinBright, MAX_BRIGHTNESS);

            if( ((LowMax - lLow) <= this->RunningAverageDiff * 2.55f) && (LowMax >= HighestLowValue / 2) && (rca.getBandAvg(0, 3) > 10)   && (rca.getBandAvg(0, 3) > LowMax*0.65f)){
                FastLED.setBrightness((uint8_t) Mathematics::Constrain((float)((currentBrightess + 60)*Rmult), this->MinBright, MAX_BRIGHTNESS));
                for(int num = 0; num < NUM_LEDS; num++){
                    CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 0.2f);
                    RGBColor refpixel = sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D());
                    pixel.r = (uint8_t) Mathematics::Constrain((Rmult * refpixel.R * UMult), 0, MAX_BRIGHTNESS / 4);
                    pixel.g = (uint8_t) Mathematics::Constrain((Gmult * refpixel.G * UMult), 0, MAX_BRIGHTNESS / 4);
                    pixel.b = (uint8_t) Mathematics::Constrain((Bmult * refpixel.B * UMult), 0, MAX_BRIGHTNESS / 4);
                    smoothing(num, pixel);
                }
                long now = millis();
                if (now - prevT >= pulseTimeoutDuration){
                    StartPulse(trans_white_pixel, 60, 2100, now-prevT);
                    prevT = millis();
                }
            }        
            else if( ((LowMax - lLow) > this->RunningAverageDiff * 2.5f) && ((LowMax - lLow) < this->RunningAverageDiff * 3.1f) && (rca.getBandAvg(0, 3) > 10)  && (rca.getBandAvg(0, 3) <= LowMax*0.65f) ){
                FastLED.setBrightness((uint8_t) Mathematics::Constrain(MinBright + currentBrightess*Rmult/15, this->MinBright, MAX_BRIGHTNESS / 2));

                for(int num = 0; num < NUM_LEDS; num++){
                    CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 0.2f);
                    RGBColor refpixel = sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D());
                    pixel.r = (uint8_t) Mathematics::Constrain((Rmult * refpixel.R * UMult), 0, MAX_BRIGHTNESS / 5);
                    pixel.g = (uint8_t) Mathematics::Constrain((Gmult * refpixel.G * UMult), 0, MAX_BRIGHTNESS / 5);
                    pixel.b = (uint8_t) Mathematics::Constrain((Bmult * refpixel.B * UMult), 0, MAX_BRIGHTNESS / 5);
                    smoothing(num, pixel);
                }
            }
            else{
                if (rca.getBandAvg(10,16) > 20 || rca.getBandAvg(0,3) > 5 || rca.getBandAvg(3,10) > 2) {
                    FastLED.setBrightness((uint8_t)(Mathematics::Constrain(MinBright + currentBrightess*Rmult/20, this->MinBright, MAX_BRIGHTNESS / 3)));
                }

                if (FastLED.getBrightness() > MinBright) FastLED.setBrightness(FastLED.getBrightness()-1);

                int sat = 0.8f;

                for(int num = 0; num < NUM_LEDS; num++){
                    CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 0.1f);
                    RGBColor refpixel = sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D());
                    float gray = GrayScale(pixel.r, pixel.g, pixel.b, sat);

                    if (rca.getBandAvg(0, 3) > 10){
                        pixel.r = (uint8_t) Mathematics::Constrain(Rmult * refpixel.R * UMult, 60, MAX_BRIGHTNESS / 6);
                    }
                    else pixel.r = (uint8_t) Mathematics::Constrain((x*UMult*refpixel.R)*(1-sat) + gray, 60, MAX_BRIGHTNESS / 6);
                    if (rca.getBandAvg(4, 10) > 10){
                        pixel.g = (uint8_t) Mathematics::Constrain((Gmult * (refpixel.G) * UMult), 50, MAX_BRIGHTNESS / 6);
                    }
                    else pixel.g = (uint8_t) Mathematics::Constrain((y*UMult*refpixel.G)*(1-sat) + gray, 50, MAX_BRIGHTNESS / 6);
                    if (rca.getBandAvg(10, 16) > 15){
                        pixel.b = (uint8_t) Mathematics::Constrain((Bmult * (refpixel.B) * UMult), 60, MAX_BRIGHTNESS / 6);
                    }
                    else pixel.b = (uint8_t) Mathematics::Constrain((z*UMult*refpixel.B)*(1-sat) + gray, 60, MAX_BRIGHTNESS / 6);
                    smoothing(num, pixel);
                }

            }
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
                // Smoothing + Amplification
                const float brightSmooth = 0.3f;  // strong pumping feel (adjust as desired)

                // Get the brightness your audio logic produced this frame
                uint8_t target = FastLED.getBrightness();

                // --- Smooth toward target ---
                float smoothed = prevBrightness + (target - prevBrightness) * brightSmooth;

                // --- Normalize & Amplify Full Range ---
                float normalized = (smoothed - MinBright) / (float)(MAX_BRIGHTNESS - MinBright);
                normalized = Mathematics::Constrain(normalized, 0.0f, 1.0f);

                // Remap back to full usable brightness span
                uint8_t amplifiedBrightness = (uint8_t)(MinBright + normalized * (MAX_BRIGHTNESS - MinBright));

                // Apply and store for next frame
                FastLED.setBrightness(amplifiedBrightness);
                prevBrightness = amplifiedBrightness;
            };

            rca.Sample();
            rca.FFT();

            float low  = rca.getBandAvg(0,3);
            float high = rca.getBandAvg(10,16);

            static bool noAudio = false;
            static uint32_t silenceTimer = 0;

            // If all bands are consistently below noise floor → silence
            if (low < 10.0f && high < 20.0f) {
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
            }

            RenderPulses();
            smoothBright();
            FastLED.show();
        }
};