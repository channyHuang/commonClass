#ifndef VERTEX_H
#define VERTEX_H

#ifndef real
#define real float
#endif

#include "vector2.h"

class Vertex2D {
public:
    Vertex2D() {}
    Vertex2D(real x, real y) : vPos(x, y) {}

    Vector2 vPos;
    int incidentEdge = -1;
    int id = -1;
};


#endif