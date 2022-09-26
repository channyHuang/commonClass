#ifndef PLANE_H
#define PLANE_H

#include "vector3.h"

class Plane {
public:
    Plane() : vNormal_(Vector3(0, 1, 0)), vPoint_(Vector3(0)) {}
    Plane(const Vector3& _vNormal, const Vector3& _vPoint) : vNormal_(_vNormal), vPoint_(_vPoint) {}

    Vector3 vNormal_;
    Vector3 vPoint_;
};

#endif
