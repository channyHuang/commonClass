#ifndef PLANE_H
#define PLANE_H

#include "vector3.h"

class Plane {
public:
    Plane() : vNormal_(Vector3(0, 1, 0)), vPoint_(Vector3(0)) {}
    Plane(const Vector3& _vNormal, const Vector3& _vPoint) : vNormal_(_vNormal), vPoint_(_vPoint) {
        vNormal_.normalize();
    }

    float distance(const Vector3& point) {
        Vector3 dir = (point - vPoint_);
        Vector3 dirNorm = dir.getNormalize();
        return dir.len() * std::fabs(dirNorm.dot(vNormal_));
    }

    Vector3 interate(const Vector3& a, const Vector3& b) {
        float da = distance(a);
        float db = distance(b);

        return a + (b - a) * da / (da + db);
    }

public:
    Vector3 vNormal_;
    Vector3 vPoint_;
};

#endif
