#pragma once
#include <Arduino.h>

class ClockMicros{
    private:
        unsigned long prevTime = 0;
    public:
        ClockMicros(){
            prevTime = micros();
        }
        bool microsDelay(int delayMicros){
            while(micros() - prevTime < delayMicros){}
            prevTime = micros();
            return true;
        }

};

class ClockMillis{
    private:
        unsigned long prevTime = 0;
    public:
        ClockMillis(){
            prevTime = millis();
        }
        bool millissDelay(int delayMillis){
            while(micros() - prevTime < delayMillis){}
            prevTime = micros();
            return true;
        }

};