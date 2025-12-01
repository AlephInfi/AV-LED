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

#define DEBUG

#define MAX_BRIGHTNESS    255 //Maximum brightness

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
        int HighestHighValue = 0;

        unsigned long prevmillis = 0;
        int ct = 0;
        int lgct = 0;
        int MultiplierRandY = random(1, 5);
        int MultiplierRandZ = random(1, 5);
        bool recvSound = false;

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

        // --------------------------------------------------
        // Brightness Pulse System
        // --------------------------------------------------
        struct BrightnessPulseData {
            bool active = false;
            float intensity = 0.0f;      // actual decaying pulse value
            float attackAmount = 0.0f;   // how much to add instantly
            float decayRate = 0.90f;     // exponential falloff
            uint32_t startTime = 0;
        };

        BrightnessPulseData brightnessPulse;


        // User-tunable pulse response
        const float PULSE_ATTACK  = 1.8f;   // how fast it rises when triggered
        const float PULSE_DECAY   = 0.92f;  // exponential fade each frame
        const uint16_t PULSE_MIN_GAP = 50;  // minimum ms between pulses

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

        struct BeatEvent {
            bool kick = false;
            bool snare = false;
            bool hihat = false;
        };

        BeatEvent detectBeats() {
            // --- Tunable parameters ---
            static const float SMOOTH_LOW   = 0.90f;
            static const float SMOOTH_MID   = 0.92f;
            static const float SMOOTH_HIGH  = 0.93f;

            static const float THRESH_LOW   = 1.15f;
            static const float THRESH_TRANS = 1.00f;   // transient (mid/high) for kick
            static const float THRESH_SNARE = 1.0f;
            static const float THRESH_HIHAT = 1.1f;

            static const uint16_t COOLDOWN_KICK  = 30;  // ms
            static const uint16_t COOLDOWN_SNARE = 500;
            static const uint16_t COOLDOWN_HIHAT = 80;

            // --- Persistent running averages ---
            static float avgLow = 0, avgMid = 0, avgHigh = 0;
            static uint32_t lastKick = 0, lastSnare = 0, lastHihat = 0;
            static bool initialized = false;

            if (!initialized) {
                avgLow  = rca.getBandAvg(0, 3) * rca.getBandAvg(0, 3);    // 30–120 Hz  → bass
                avgMid  = rca.getBandAvg(4, 10);    // 200–800 Hz → snare body
                avgHigh = rca.getBandAvg(11, 15);  // 2–6 kHz    → hi-hat/transients
                initialized = true;
            }

            // --- Read current energies ---
            float lowNow  = rca.getBandAvg(0, 3) * rca.getBandAvg(0, 3);
            float midNow  = rca.getBandAvg(3, 8);
            float highNow = rca.getBandAvg(8, 15);

            // --- Exponential moving averages ---
            avgLow  = SMOOTH_LOW  * avgLow  + (1.0f - SMOOTH_LOW)  * lowNow;
            avgMid  = SMOOTH_MID  * avgMid  + (1.0f - SMOOTH_MID)  * midNow;
            avgHigh = SMOOTH_HIGH * avgHigh + (1.0f - SMOOTH_HIGH) * highNow;

            uint32_t now = millis();
            BeatEvent e;

            // ===================================================
            // 🎵 Kick (dual-band: low + transient click)
            // ===================================================
            float lowThresh  = avgLow  * THRESH_LOW;
            float transNow   = rca.getBandAvg(4, 8);         // transient band (~1–5 kHz)
            static float avgTrans = 0;
            avgTrans = SMOOTH_MID * avgTrans + (1.0f - SMOOTH_MID) * transNow;
            float transThresh = avgTrans * THRESH_TRANS;

            if (lowNow > lowThresh && transNow > transThresh && now - lastKick > COOLDOWN_KICK) {
                e.kick = true;
                lastKick = now;
            }

            // ===================================================
            // 🥁 Snare (mid-range burst)
            // ===================================================
            float midThresh = avgMid * THRESH_SNARE;
            if (midNow > midThresh && now - lastSnare > COOLDOWN_SNARE) {
                e.snare = true;
                lastSnare = now;
            }

            // ===================================================
            // ✨ Hi-hat (high-frequency transient)
            // ===================================================
            float highThresh = avgHigh * THRESH_HIHAT;
            if (highNow > highThresh && now - lastHihat > COOLDOWN_HIHAT) {
                e.hihat = true;
                lastHihat = now;
            }

            return e;
        }

        /*
        Mode 0: head moves 0 → NUM_LEDS-1

        Mode 1: head moves NUM_LEDS-1 → 0

        Mode 2:
        Starts at center → expands outward

        Mode 3:
        Starts at ends → moves inward meeting in the middle
        */
        void RenderPulses(uint8_t mode) {
            uint32_t now = millis();

            auto DrawOnePulse = [&](float headPos, float length, const CRGB &color, bool forward) {
                // forward=true  → head moving toward increasing index (0 → N)
                // forward=false → head moving toward decreasing index (N → 0)

                int start = (int)headPos;
                int end   = forward ? (int)(headPos - length) : (int)(headPos + length);

                int step  = forward ? -1 : +1;

                for (int ledIndex = start; ledIndex != end + step; ledIndex += step) {
                    if (ledIndex < 0 || ledIndex >= NUM_LEDS) continue;

                    float d = fabsf(headPos - ledIndex) / length;  
                    d = Mathematics::Constrain(d, 0.0f, 1.0f);

                    const float headSharpness = 4.0f;
                    const float tailFalloff   = 3.5f;

                    float intensity = expf(-tailFalloff * d);
                    float colorBoost = expf(-headSharpness * d);

                    intensity *= (1.0f + 0.5f * colorBoost);
                    intensity = Mathematics::Constrain(intensity, 0.0f, 1.0f);

                    CRGB faded(
                        color.r * intensity,
                        color.g * intensity,
                        color.b * intensity
                    );

                    led[ledIndex].r = Mathematics::Constrain((unsigned int)led[ledIndex].r + faded.r, 0, 255);
                    led[ledIndex].g = Mathematics::Constrain((unsigned int)led[ledIndex].g + faded.g, 0, 255);
                    led[ledIndex].b = Mathematics::Constrain((unsigned int)led[ledIndex].b + faded.b, 0, 255);
                }
            };

            for (uint8_t i = 0; i < pulseCount; i++) {
                Pulse &p = pulses[i];

                float progress = (float)(now - p.startTime) / (float)p.duration;

                if (progress >= 1.0f) {
                    // remove finished pulse
                    for (uint8_t j = i; j < pulseCount - 1; j++) pulses[j] = pulses[j + 1];
                    pulseCount--;
                    i--;
                    continue;
                }

                float headPos = progress * (NUM_LEDS - 1);  // 0 → N-1
                float length  = p.length;

                switch (mode) {

                // ------------------------------------------------------------
                // MODE 0: FROM_START (left → right)
                // ------------------------------------------------------------
                case 0: {
                    DrawOnePulse(headPos, length, p.color, /*forward*/ true);
                    break;
                }

                // ------------------------------------------------------------
                // MODE 1: FROM_END (right → left)
                // ------------------------------------------------------------
                case 1: {
                    float hp = (NUM_LEDS - 1) - headPos;
                    DrawOnePulse(hp, length, p.color, /*forward*/ false);
                    break;
                }

                // ------------------------------------------------------------
                // MODE 2: FROM_MIDDLE (center → outward)
                // Two mirrored pulses
                // ------------------------------------------------------------
                case 2: {
                    float center = (NUM_LEDS - 1) * 0.5f;
                    float offset = headPos * 0.5f;

                    float leftHead  = center - offset;
                    float rightHead = center + offset;

                    DrawOnePulse(leftHead,  length, p.color, /*forward*/ false);  // left pulse moves left
                    DrawOnePulse(rightHead, length, p.color, /*forward*/ true);   // right pulse moves right
                    break;
                }

                // ------------------------------------------------------------
                // MODE 3: FROM_ENDS (ends → center)
                // Two pulses moving inward toward the middle
                // ------------------------------------------------------------
                case 3: {
                    float center = (NUM_LEDS - 1) * 0.5f;

                    float leftHead  = headPos;                    // starts at 0 and moves toward center
                    float rightHead = (NUM_LEDS - 1) - headPos;   // starts at N and moves toward center

                    DrawOnePulse(leftHead,  length, p.color, /*forward*/ true);   // left moves right
                    DrawOnePulse(rightHead, length, p.color, /*forward*/ false);  // right moves left
                    break;
                }

                } // end switch
            }
        }


        void StartBrightnessPulse(float attack, float decay)
        {
            brightnessPulse.active = true;
            brightnessPulse.attackAmount = attack;  // how strong the “pop” is
            brightnessPulse.decayRate = decay;      // typically 0.90–0.97
            brightnessPulse.intensity += attack;
            if (brightnessPulse.intensity > 1.0f)
                brightnessPulse.intensity = 1.0f;

            brightnessPulse.startTime = millis();
        }

        float UpdateBrightnessPulse()
        {
            if (!brightnessPulse.active)
                return 0.0f;

            // Exponential decay
            brightnessPulse.intensity *= brightnessPulse.decayRate;

            // Stop when nearly zero
            if (brightnessPulse.intensity < 0.001f) {
                brightnessPulse.intensity = 0.0f;
                brightnessPulse.active = false;
            }

            return brightnessPulse.intensity;   // caller adds this to brightness
        }



        void StartPulse(CRGB color, float length, uint32_t t_ms, uint16_t timeSince = 300) {
            if (pulseCount >= MAX_PULSES) return; // no room, ignore
            if (timeSince < 150) return; // max supported at eigth notes at 200bpm

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
                const float smoothFactor = 0.2f;  

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
            
            static unsigned long prevkick = millis();
            static unsigned long prevsnr = millis();
            //static unsigned long prevhh = millis();

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

                int sat = 0.9f;

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

            long now = millis();
            BeatEvent ev = detectBeats();
            if (ev.snare && now-prevsnr > 500){
                StartBrightnessPulse(1.0f, 0.7f);
                prevsnr = millis();
            }
            if (ev.kick && now-prevkick > 30 && lLow > LowMax * 0.6f){
                StartPulse(trans_white_pixel, 60, 1050, now-prevkick);
                prevkick = millis();
            }
            if( ((LowMax - lLow) <= this->RunningAverageDiff * 2.55f) && (LowMax >= HighestLowValue / 2) && (rca.getBandAvg(0, 3) > 10)   && (rca.getBandAvg(0, 3) > LowMax*0.65f)){
                FastLED.setBrightness((uint8_t) Mathematics::Constrain((float)(currentBrightess*Rmult), this->MinBright, MAX_BRIGHTNESS));
                for(int num = 0; num < NUM_LEDS; num++){
                    CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 0.2f);
                    RGBColor refpixel = sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D());
                    pixel.r = (uint8_t) Mathematics::Constrain((Rmult * refpixel.R * UMult) / 3, 0, 190);
                    pixel.g = (uint8_t) Mathematics::Constrain((Rmult * refpixel.G * UMult) / 3, 0, 190);
                    pixel.b = (uint8_t) Mathematics::Constrain((Rmult * refpixel.B * UMult) / 3, 0, 190);
                    smoothing(num, pixel);
                }
            }        
            else if( ((LowMax - lLow) > this->RunningAverageDiff * 2.5f) && ((LowMax - lLow) < this->RunningAverageDiff * 3.1f) && (rca.getBandAvg(0, 3) > 10)  && (rca.getBandAvg(0, 3) <= LowMax*0.65f) ){
                FastLED.setBrightness((uint8_t) Mathematics::Constrain(MinBright + currentBrightess*Rmult/15, this->MinBright, MAX_BRIGHTNESS / 2));

                for(int num = 0; num < NUM_LEDS; num++){
                    CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 0.2f);
                    RGBColor refpixel = sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D());
                    pixel.r = (uint8_t) Mathematics::Constrain((Rmult * refpixel.R * UMult)/7, 0, 150);
                    pixel.g = (uint8_t) Mathematics::Constrain((Gmult * refpixel.G * UMult)/7, 0, 150);
                    pixel.b = (uint8_t) Mathematics::Constrain((Bmult * refpixel.B * UMult)/7, 0, 150);
                    smoothing(num, pixel);
                }
            }
            else{
                if (rca.getBandAvg(10,16) > 20 || rca.getBandAvg(0,3) > 5 || rca.getBandAvg(3,10) > 2) {
                    FastLED.setBrightness((uint8_t)(Mathematics::Constrain(MinBright + currentBrightess*Rmult/20, this->MinBright, MAX_BRIGHTNESS / 4)));
                }
                static int sat = 0.8f;

                for(int num = 0; num < NUM_LEDS; num++){
                    CRGB pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()), 0.1f);
                    RGBColor refpixel = sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D());
                    float gray = GrayScale(pixel.r, pixel.g, pixel.b, sat);

                    if (rca.getBandAvg(0, 3) > 10){
                        pixel.r = (uint8_t) Mathematics::Constrain(Rmult * refpixel.R * UMult, 0, 100);
                    }
                    else pixel.r = (uint8_t) Mathematics::Constrain((x*UMult*refpixel.R)*(1-sat) + gray, 0, 100);
                    if (rca.getBandAvg(4, 10) > 10){
                        pixel.g = (uint8_t) Mathematics::Constrain((Gmult * (refpixel.G) * UMult), 0, 100);
                    }
                    else pixel.g = (uint8_t) Mathematics::Constrain((y*UMult*refpixel.G)*(1-sat) + gray, 0, 100);
                    if (rca.getBandAvg(10, 16) > 15){
                        pixel.b = (uint8_t) Mathematics::Constrain((Bmult * (refpixel.B) * UMult), 0, 100);
                    }
                    else pixel.b = (uint8_t) Mathematics::Constrain((z*UMult*refpixel.B)*(1-sat) + gray, 0, 100);
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
            static int silCt = 0;
            auto smoothBright = [&] (){
                // Smoothing + Amplification
                const float brightSmooth = 0.4f;  // strong pumping feel (adjust as desired)

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

            lLow = rca.getBandAvg(0,3);
            float mid = rca.getBandAvg(4, 9);
            float high = rca.getBandAvg(10,16);

            static bool noAudio = false;
            static uint32_t silenceTimer = 0;

            #ifdef DEBUG
            //Serial.print(lLow); Serial.print("\t"); Serial.print(mid); Serial.print("\t"); Serial.println(high);
            //Serial.print(HighestLowValue); Serial.print("\t"); Serial.print(HighestHighValue); Serial.print("\t"); Serial.println(noAudio);
            #endif

            if (ct >= 50) {
                ct = 0;
            }

            if(lLow > LowMax) LowMax = lLow;
            if(mid > MidMax) MidMax = mid;
            if(high > HighMax) HighMax = high;

            if(millis() - prevmillis >= 5000){
                LowMax = 0.1f;
                MidMax = 0.1f;
                HighMax = 0.1f;

                if (high > HighestHighValue) HighestHighValue = high;
                if (lLow > HighestLowValue) HighestLowValue = lLow;
                prevmillis = millis();
            }

            if(silCt < 10) {
                if(lLow > 150 || mid > 150 || high > 150) silCt++;
            }
            if(silCt >= 10) recvSound = true;

            // If all bands are consistently below noise floor → silence
            
            if ((lLow < (HighestLowValue * 0.3f) && high < (HighestHighValue * 0.4f)) || !recvSound) {
                if (millis() - silenceTimer > 120) noAudio = true;
            } else {
                silenceTimer = millis();
                noAudio = false;
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
            
            RenderPulses(2);
            smoothBright();
            // 2. Each frame, get the boost
            float boost = UpdateBrightnessPulse(); // 0–1

            // 3. Add it to your rendered brightness
            uint8_t base = FastLED.getBrightness();
            uint8_t added = base + (uint8_t)(boost * MAX_BRIGHTNESS); // boost amount
            if (added > MAX_BRIGHTNESS) added = MAX_BRIGHTNESS;
            if (added < MinBright) added = MinBright;

            FastLED.setBrightness(added);
            FastLED.show();
        }
};