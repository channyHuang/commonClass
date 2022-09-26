#ifndef FRUSTUM_H
#define FRUSTUM_H

#include "plane.h"

class Frustum {
public:
    Frustum() {}

    Plane plane_[6];
};

#endif
