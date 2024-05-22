#pragma once

#include "MiniBpm.h"
#include "RCAIn.h"
#include "Clock.h"

#define SECS    5 //number of seconds to analyze

class BPM{
    private:
        MiniBPM bpm = MiniBPM(SAMPLE_RATE);
        double CurrentBPM = 140; //common BPM to start with
        ClockMillis timer;

    public:
        BPM(){};

        void Update(RCA rca){
            unsigned long p = millis();
            while(millis() - p < SECS * 1000){
                bpm.process(rca.getAmpMono(), NUM_SAMPLES);
                timer.millissDelay(rca.getMicrosDelay() * 1000);
                CurrentBPM = bpm.estimateTempo();
            }
        }

        //return bpm estimate
        double GetBPM(){
            return CurrentBPM;
        }
};