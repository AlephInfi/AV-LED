#pragma once

#include <NeoPixelBus.h>
#include "..\Audio\BPMDetect.h"

#include "..\Materials\SimplexNoise.h"

/* for 4-pin LEDs
#define LEDCLK          14
#define LEDDAT          2
*/
#define LED_VOLTS       5
#define LEDPIN          13
#define NUM_LEDS        1

#define PULSE_SCALAR    10 //Controls the brightness increase of LEDs when pulsing

class LEDStrip{
    private:
        RgbColor led[NUM_LEDS];

        RCA rca;
        BPM bpmDetect;
        NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1X8Ws2812xMethod> LED = NeoPixelBus<NeoGrbFeature, NeoEsp32I2s1X8Ws2812xMethod>(NUM_LEDS, LEDPIN);

        RGBColor noiseSpectrum[4] = {RGBColor(0, 255, 0), RGBColor(255, 0, 0), RGBColor(0, 255, 0), RGBColor(0, 0, 255)};
        GradientMaterial gNoiseMat = GradientMaterial(4, noiseSpectrum, 2.0f, false);
        SimplexNoise sNoise = SimplexNoise(1, &gNoiseMat);

        RgbColor RGBColorToRgb(RGBColor color){ //NeoBus method
            RgbColor crgb;
            crgb.R = color.R;
            crgb.G = color.G;
            crgb.B = color.B;
            return crgb;
        }

        //Function states:
        bool SimplexState = false;

        void On(bool index){
            index = true;
        }

        void Off(bool index){
            index = false;
        }

        void Simplex(float ratio){
            if(SimplexState = true){
                float x = 0.5f * sinf(ratio * 3.14159f / 180.0f * 360.0f * 2.0f);
                float y = 0.5f * cosf(ratio * 3.14159f / 180.0f * 360.0f * 3.0f);

                float linSweep = ratio > 0.5f ? 1.0f - ratio : ratio;
                float sShift = linSweep * 0.002f + 0.005f;

                gNoiseMat.SetGradientPeriod(0.5f + linSweep * 6.0f);
                gNoiseMat.HueShift(ratio * 360 * 2);
                sNoise.SetScale(Vector3D(sShift, sShift, sShift));
                sNoise.SetZPosition(x * 4.0f);

                for(int num = 0; num < NUM_LEDS; num++){
                    RgbColor pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()));
                    led[num] = pixel;
                }
            }
        }

        //Use this for pulse amp? uint scalar = Mathematics::Constrain(((uint)Mathematics::Map(rca.Gain, -60.0f, 5.0f, 0.0f, 255.0f)), 0, 255);

        void Pulse(uint8_t amplitude){
            int t = 0;
            int beatGap = (int)(60000.0f / bpmDetect.GetBPM()); // in milliseconds
            RgbColor savedLEDState[NUM_LEDS];
            for(int num = 0; num < NUM_LEDS; num++){
                RgbColor pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()));
                savedLEDState[num] = pixel;
                pixel.Lighten(amplitude);
                led[num] = pixel;
            }
            while(t < beatGap){
                for(int num = 0; num < NUM_LEDS; num++){
                    RgbColor pixel = RGBColorToRgb(sNoise.GetRGB(Vector3D(num,num,num), Vector3D(), Vector3D()));
                    pixel.Lighten((-1 * amplitude / beatGap));
                    led[num] = pixel;
                }
                t += 1; // millisecond
            }
        }

    public:
        LEDStrip(){}

        void Init(){
            LED.Begin();
        }

        void Update(int command, float ratio){
            rca.Sample();
            rca.FFT();
            bpmDetect.Update(rca);

            if(command == 0) On(SimplexState);
            if(command == 1) Off(SimplexState);
bool tmp; // placehold
            if(command == 2) On(tmp);
            if(command == 3) Off(tmp);

            if(command == 4) On(tmp);
            if(command == 5) Off(tmp);

            if(command == 6) On(tmp);
            if(command == 7) Off(tmp);
            
            LED.Show();
        }
};