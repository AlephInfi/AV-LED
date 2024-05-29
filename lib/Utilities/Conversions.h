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

CRGB RGBColorToRgb(RGBColor color){ //NeoBus method
            CRGB crgb;
            crgb.r = color.R;
            crgb.g = color.G;
            crgb.b = color.B;
            return crgb;
        }