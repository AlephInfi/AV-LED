#pragma once

#include "..\Math\Mathematics.h"
#include "..\Materials\RGBColor.h"
#include <FastLED.h>

float* doubToFloatArr(double array[]){
    float ret[(int)(sizeof(array)/sizeof(array[0]))];
    for(int i = 0; i < (int)(sizeof(array)/sizeof(array[0])); i++){
        ret[i] = (float)array[i];
    }
    return ret;
}

CRGB RGBColorToRgb(RGBColor color, float multiplier = 1.0f){ //NeoBus method
            CRGB crgb;
            crgb.r = round(color.R * multiplier);
            crgb.g = round(color.G * multiplier);
            crgb.b = round(color.B * multiplier);
            return crgb;
        }