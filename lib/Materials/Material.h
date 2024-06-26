#pragma once

#include "..\Materials\RGBColor.h"
#include "..\Math\Vector3D.h"

class Material{
public:
    virtual RGBColor GetRGB(Vector3D position, Vector3D normal, Vector3D uvw) = 0;
  
};
