#pragma once
#include <arduinoFFT.h>
#include "..\Utilities\Clock.h"

#define LEFT_PIN        22
#define RIGHT_PIN       23

#define SAMPLE_RATE     100000   // 100 kHz
#define NUM_SAMPLES     1024
#define BINS            16
#define SENSITIVITY     1

class RCA{
    private:
        static ClockMicros microsTimer;
        static unsigned int sampling_period_us;
        static byte peak[BINS];
        static int oldBarHeights[BINS];
        static int bandValues[BINS];

        float vRealL[NUM_SAMPLES];
        float vImagL[NUM_SAMPLES];
        ArduinoFFT<float> FFTLeft = ArduinoFFT<float>(vRealL, vImagL, NUM_SAMPLES, SAMPLE_RATE);

        double AvGainL;

    public:
        RCA(){}

        void SetupADC(){
            analogReadResolution(12);    // Teensy 4.0: 12-bit ADC
            analogReadAveraging(1);      // no averaging, faster reads
        }

        void Sample(){
            AvGainL = 0;
            for(int i = 0; i < NUM_SAMPLES; i++){
                vRealL[i] = analogRead(LEFT_PIN) / 200.0f;
                vImagL[i] = 0;
                AvGainL += vRealL[i];

                // Wait until next sample period
                uint32_t start = micros();
                while (micros() - start < sampling_period_us);
            }
            AvGainL /= NUM_SAMPLES;
        }

        float* getAmpL(){ return vRealL; }
        double getGainL(double multiplier = 1.0f){ return AvGainL * multiplier; }
        int getBandAvg(int MinBins = 0, int MaxBins = BINS - 1){
            if (MaxBins > BINS - 1) MaxBins = BINS - 1;
            int total = 0;
            for (int i = MinBins; i <= MaxBins; i++){
                total += bandValues[i];
            }
            return (int)(total / (MaxBins - MinBins + 1));
        }

        void FFT(){
            for (int i = 0; i < BINS; i++) bandValues[i] = 0;

            FFTLeft.dcRemoval(vRealL, NUM_SAMPLES);
            FFTLeft.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
            FFTLeft.compute(FFT_FORWARD);
            FFTLeft.complexToMagnitude();

            for (int i = 2; i < (NUM_SAMPLES / 2); i++){
                if (i<=2 )           bandValues[0]  += (int)vRealL[i];
                else if (i<=3)       bandValues[1]  += (int)vRealL[i];
                else if (i<=5)       bandValues[2]  += (int)vRealL[i];
                else if (i<=7)       bandValues[3]  += (int)vRealL[i];
                else if (i<=9)       bandValues[4]  += (int)vRealL[i];
                else if (i<=13)      bandValues[5]  += (int)vRealL[i];
                else if (i<=18)      bandValues[6]  += (int)vRealL[i];
                else if (i<=25)      bandValues[7]  += (int)vRealL[i];
                else if (i<=36)      bandValues[8]  += (int)vRealL[i];
                else if (i<=50)      bandValues[9]  += (int)vRealL[i];
                else if (i<=69)      bandValues[10] += (int)vRealL[i];
                else if (i<=97)      bandValues[11] += (int)vRealL[i];
                else if (i<=135)     bandValues[12] += (int)vRealL[i];
                else if (i<=189)     bandValues[13] += (int)vRealL[i];
                else if (i<=264)     bandValues[14] += (int)vRealL[i];
                else                 bandValues[15] += (int)vRealL[i];
            }

            for (byte band = 0; band < BINS; band++) {
                int barHeight = bandValues[band] / SENSITIVITY;
                barHeight = ((oldBarHeights[band] * 1) + barHeight) / 2;
                if (barHeight > peak[band]) peak[band] = barHeight;
                oldBarHeights[band] = barHeight;
            }
        }

        int getBand(int band){ return bandValues[band]; }
        int* getBandArr(){ return bandValues; }
};

ClockMicros RCA::microsTimer = ClockMicros();
unsigned int RCA::sampling_period_us = round(1000000.0 / SAMPLE_RATE);
int RCA::bandValues[BINS] = {0};
int RCA::oldBarHeights[BINS] = {0};
byte RCA::peak[BINS] = {0};
