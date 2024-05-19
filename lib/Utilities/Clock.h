#pragma once
#include <Arduino.h>

class ClockMicros{
    private:
        unsigned long time = 0;
        unsigned long prevTime = 0;
    public:
        ClockMicros(){
            prevTime = micros();
        }
        void microsDelay(int delayMicros){
            while(time - prevTime < delayMicros){
                time = micros();
            }
            prevTime = time;
        }

};

class ClockMillis{
    private:
        unsigned long time = 0;
        unsigned long prevTime = 0;
    public:
        ClockMillis(){
            prevTime = millis();
        }
        void millissDelay(int delayMillis){
            while(time - prevTime < delayMillis){
                time = millis();
            }
            prevTime = time;
        }

};