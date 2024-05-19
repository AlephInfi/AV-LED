#include <Arduino.h>
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
#define LEFT_PIN -1
#define RIGHT_PIN -1
#define LEDCLK -1
#define LEDDAT -1

#define SAMPLE_RATE     28000 //gives frequencies up to 18kHz
#define NUM_SAMPLES     (SAMPLE_RATE / 1000) //samples in one millisecond
#define BINS            64 //rather high
#define SENSITIVITY     1000

class RCA{
    private:
        static ClockMicros microsTimer;
    public:
        static float gainR[NUM_SAMPLES];
        static float gainL[NUM_SAMPLES];

        RCA(){
        }

        static void Sample(){
            for(int i = 0; i < NUM_SAMPLES; i++){
                gainR[i] = analogReadRaw(RIGHT_PIN);
                gainL[i] = analogReadRaw(LEFT_PIN);
                microsTimer.microsDelay(1);
            }
        }

        static void analyze(float input[]){

        }
};