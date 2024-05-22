#pragma once
#include <Wire.h>
#include <BluetoothSerial.h>

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

class Bluetooth{
    private:
        BluetoothSerial SerialBT;
        int BTValue = 0;

    public:
    //operating on 115200 baud rate
        Bluetooth(){}

        void BTBegin(){
            SerialBT.begin("AV-LED");
        }

        void Update(){
            //debug
            if (Serial.available()) {
                SerialBT.write(Serial.read());
            }
            if (SerialBT.available()) {
                Serial.write(SerialBT.read());
                BTValue = (int)(SerialBT.read()); //take in integers
            }
        }

        uint8_t getValue(){
            return BTValue;
        }
};