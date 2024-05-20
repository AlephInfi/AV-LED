#pragma once
#include <Arduino.h>

class ClockMicros{
    private:
        unsigned long prevTime = 0;
    public:
        ClockMicros(){
            prevTime = micros();
        }
        void microsDelay(int delayMicros){
            while(micros() - prevTime < delayMicros){}
            prevTime = micros();
        }

};

class ClockMillis{
    private:
        unsigned long prevTime = 0;
    public:
        ClockMillis(){
            prevTime = millis();
        }
        void millissDelay(int delayMillis){
            while(micros() - prevTime < delayMillis){}
            prevTime = micros();
        }

};