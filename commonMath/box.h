#pragma once

#include "vector3.h"

class Box
{
public:
    Box() { vMin = Vector3(0), vMax = Vector3(0); };
    Box(const Vector3 &_vMin, const Vector3 &_vMax) : vMin(_vMin), vMax(_vMax) {}
    Box(float fminx, float fminy, float fminz, float fmaxx, float fmaxy, float fmaxz)
        : vMin(fminx, fminy, fminz), vMax(fmaxx, fmaxy, fmaxz) {}
    ~Box() {};

    Vector3 getCenter() const { return (vMin + vMax) * 0.5f; }
    float getDX() { return vMax.x - vMin.x; }
    float getDY() { return vMax.y - vMin.y; }
    float getDZ() { return vMax.z - vMin.z; }

    bool isPointInside(const Vector3 &vpos) {
        return (vpos.x > vMin.x && vpos.x < vMax.x
                && vpos.y > vMin.y && vpos.y < vMax.y
                && vpos.z > vMin.z && vpos.z < vMax.z);
    }
public:
    Vector3 vMin, vMax;
};
