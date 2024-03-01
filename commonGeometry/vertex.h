#ifndef VERTEX_H
#define VERTEX_H

#include "commonMath/vector2.h"

class Vertex2D {
public:
    Vertex2D() {}
    Vertex2D(float x, float y) : vPos(x, y) {}

    Vector2 vPos;
    int incidentEdge = -1;
    int id = -1;
};


#endif
