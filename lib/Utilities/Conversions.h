#pragma once

#include "..\Math\Mathematics.h"
#include "..\Materials\RGBColor.h"
#include <FastLED.h>

CRGB RGBColorToRgb(RGBColor color, float multiplier = 1.0f){ //NeoBus method
            CRGB crgb;
            crgb.r = round(color.R * multiplier);
            crgb.g = round(color.G * multiplier);
            crgb.b = round(color.B * multiplier);
            return crgb;
        }

float* CompressArray(float inpArray[], uint16_t size){
    if (size % 2 != 0) size -= 1;
    float output[size / 2];
    for(int i = 0; i < size; i += 2){
        output[i] = inpArray[i];
    }
    return output;
}