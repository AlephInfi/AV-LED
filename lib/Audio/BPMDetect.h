#pragma once

#include "..\Utilities\MiniBpm.h"
#include "..\Utilities\Conversions.h"
#include "..\Device\RCAIn.h"

#define SECS    5 //number of seconds to analyze

class BPM{
    private:
        MiniBPM bpm = MiniBPM((float)SAMPLE_RATE);
        double CurrentBPM = 140; //common BPM to start with
        ClockMillis timer;

    public:
        BPM(){};

        void Update(RCA &rca){
            bpm.process(rca.getAmpL(), (int)NUM_SAMPLES);
            CurrentBPM = bpm.estimateTempo();
            Serial.println("BPM Success!");
        }

        void SetRange(double lowerLim = 55.0f, double upperLim = 180.0f){
            bpm.setBPMRange(lowerLim, upperLim);
        }

        //return bpm estimate
        double GetBPM(){
            return CurrentBPM;
        }
};