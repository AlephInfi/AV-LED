#pragma once
//Heavily adapted from https://github.com/G6EJD/ESP32-8266-Audio-Spectrum-Display/blob/master/ESP32_Spectrum_Display_02.ino 

#include <arduinoFFT.h>
#include "..\Utilities\Clock.h"

/** @remark
 * Information on a 3.5 audio jack for reference:
 * Tip = Left channel
 * First ring from tip: Right Channel
 * Second ring from tip: GND
 * Sleeve: Microphone
*/

//PINS
#define LEFT_PIN        22
#define RIGHT_PIN       23

#define SAMPLE_RATE     36000 //gives frequencies up to 18kHz
#define NUM_SAMPLES     1024 //Multiple of 2
#define BINS            16 //Number of bands
#define SENSITIVITY     1

class RCA{
    private:
        static ClockMicros microsTimer;

        static unsigned int sampling_period_us;
        static byte peak[BINS];              // The length of these arrays must be >= NUM_BANDS
        static int oldBarHeights[BINS];
        static int bandValues[BINS];
        float vRealL[NUM_SAMPLES];
        float vImagL[NUM_SAMPLES];
        //double vRealR[NUM_SAMPLES];
        //double vImagR[NUM_SAMPLES];
        static unsigned long newTime;
        //ArduinoFFT<double> FFTRight = ArduinoFFT<double>(vRealR, vImagR, NUM_SAMPLES, SAMPLE_RATE);
        ArduinoFFT<float> FFTLeft = ArduinoFFT<float>(vRealL, vImagL, NUM_SAMPLES, SAMPLE_RATE);

        double gainMono;
    public:
        //double AvGain;
        double AvGainL;
        //double AvGainR;

        RCA(){}

        void Sample(){
            AvGainL = 0;
            for(int i = 0; i < NUM_SAMPLES; i++){
                //vRealR[i] = analogRead(RIGHT_PIN) / 200.0f;
                vRealL[i] = analogRead(LEFT_PIN) / 200.0f;
                //AvGain += gainMono[i];
                AvGainL += vRealL[i];
                //AvGainR += gainR[i];
                vImagL[i] = 0;
                //vImagR[i] = 0;
                while (!microsTimer.microsDelay(sampling_period_us)) {};
            }
            //AvGain /= NUM_SAMPLES;
            AvGainL /= NUM_SAMPLES;
            //AvGainR /= NUM_SAMPLES;
            //Mathematics::Constrain(AvGainL, -60.0f, 5.0f);
            //Mathematics::Constrain(AvGainR, -60.0f, 5.0f);
            //Mathematics::Constrain(AvGain, -60.0f, 5.0f);
        }

        //gain raw data
        float* getAmpL(){
            return vRealL;
        }

        double getGainL(double multiplier = 1.0f){
            return AvGainL * multiplier;
        }

        float getBandAvg(){
            int total;
            for (int i = 0; i < BINS; i++){
                total += bandValues[i];
            }
            return (total / BINS);
        }

        int getMicrosDelay(){
            return sampling_period_us;
        }

        void FFT(){
            for (int i = 0; i < BINS; i++){
                bandValues[i] = 0;
            }
            FFTLeft.dcRemoval(vRealL, NUM_SAMPLES);
            FFTLeft.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
            FFTLeft.compute(FFT_FORWARD);
            FFTLeft.complexToMagnitude();

            double peakL = FFTLeft.majorPeak(vRealL, NUM_SAMPLES, SAMPLE_RATE);
            //double peakR = FFTRight.majorPeak(gainR(), NUM_SAMPLES, SAMPLE_RATE);
            
            //FFTRight.dcRemoval();
            //FFTRight.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
            //FFTRight.compute(FFT_FORWARD);
            //FFTRight.complexToMagnitude();

            for (int i = 2; i < (NUM_SAMPLES / 2); i++){
                /*8 bands, 12kHz top band
                if (i<=3 )           bandValues[0]  += (int)vReal[i];
                if (i>3   && i<=6  ) bandValues[1]  += (int)vReal[i];
                if (i>6   && i<=13 ) bandValues[2]  += (int)vReal[i];
                if (i>13  && i<=27 ) bandValues[3]  += (int)vReal[i];
                if (i>27  && i<=55 ) bandValues[4]  += (int)vReal[i];
                if (i>55  && i<=112) bandValues[5]  += (int)vReal[i];
                if (i>112 && i<=229) bandValues[6]  += (int)vReal[i];
                if (i>229          ) bandValues[7]  += (int)vReal[i];*/

                //16 bands, 12kHz top band
                if (i<=2 )           bandValues[0]  += (int)vRealL[i];
                if (i>2   && i<=3  ) bandValues[1]  += (int)vRealL[i];
                if (i>3   && i<=5  ) bandValues[2]  += (int)vRealL[i];
                if (i>5   && i<=7  ) bandValues[3]  += (int)vRealL[i];
                if (i>7   && i<=9  ) bandValues[4]  += (int)vRealL[i];
                if (i>9   && i<=13 ) bandValues[5]  += (int)vRealL[i];
                if (i>13  && i<=18 ) bandValues[6]  += (int)vRealL[i];
                if (i>18  && i<=25 ) bandValues[7]  += (int)vRealL[i];
                if (i>25  && i<=36 ) bandValues[8]  += (int)vRealL[i];
                if (i>36  && i<=50 ) bandValues[9]  += (int)vRealL[i];
                if (i>50  && i<=69 ) bandValues[10] += (int)vRealL[i];
                if (i>69  && i<=97 ) bandValues[11] += (int)vRealL[i];
                if (i>97  && i<=135) bandValues[12] += (int)vRealL[i];
                if (i>135 && i<=189) bandValues[13] += (int)vRealL[i];
                if (i>189 && i<=264) bandValues[14] += (int)vRealL[i];
                if (i>264          ) bandValues[15] += (int)vRealL[i];
            }

            for (byte band = 0; band < BINS; band++) {
                // Scale the bars for the display
                int barHeight = bandValues[band] / SENSITIVITY;

                // Small amount of averaging between frames
                barHeight = ((oldBarHeights[band] * 1) + barHeight) / 2;

                // Move peak up
                if (barHeight > peak[band]) {
                    peak[band] = barHeight;
                }
                // Save oldBarHeights for averaging later
                oldBarHeights[band] = barHeight;
            }

            /**
             * End results:
             * bandValues[] stores values of bands in each entry.
             * peak[] is just the max value of each band at a particular instance. Used for visual effects.
            */
        }

        int getBand(int band){
            return bandValues[band];
        }

        int* getBandArr(){
            return bandValues;
        }
};

ClockMicros RCA::microsTimer = ClockMicros();
unsigned int RCA::sampling_period_us = round(1000000 * (1.0 / SAMPLE_RATE));
int RCA::bandValues[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int RCA::oldBarHeights[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
byte RCA::peak[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; 