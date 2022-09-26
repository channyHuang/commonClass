#pragma once

#include "vector3.h"

class Box
{
public:
    Box() { vMin = Vector3(0); vMax = Vector3(0); vExtent = Vector3(0); };
    Box(const Vector3 &_vMin, const Vector3 &_vMax) : vMin(_vMin), vMax(_vMax), vExtent(_vMax - _vMin) {}
    Box(float fminx, float fminy, float fminz, float fmaxx, float fmaxy, float fmaxz)
        : vMin(fminx, fminy, fminz), vMax(fmaxx, fmaxy, fmaxz) {
        vExtent = vMax - vMin;
    }
    ~Box() {};

    Vector3 getCenter() const { return (vMin + vMax) * 0.5f; }
    float getDX() { return vMax.x - vMin.x; }
    float getDY() { return vMax.y - vMin.y; }
    float getDZ() { return vMax.z - vMin.z; }
    float getDMax() { return std::max(std::max(vExtent.x, vExtent.y), vExtent.z); }

    bool isPointInside(const Vector3 &vpos) {
        return (vpos.x > vMin.x && vpos.x < vMax.x
                && vpos.y > vMin.y && vpos.y < vMax.y
                && vpos.z > vMin.z && vpos.z < vMax.z);
    }

    bool intersect(const Box& box) {
        if (box.vMin.x > vMax.x || box.vMin.y > vMax.y || box.vMin.z > vMax.z
                || box.vMax.x < vMin.x || box.vMax.y < vMin.y || box.vMax.z < vMin.z) return false;
        return true;
    }
    // this inside box
    bool inside(const Box& box) {
        return (vMin.x > box.vMin.x && vMin.x < box.vMax.x
                && vMin.y > box.vMin.y && vMin.y < box.vMax.y
                && vMin.z > box.vMin.z && vMin.z < box.vMax.z);
    }
public:
    Vector3 vMin, vMax, vExtent;
};
