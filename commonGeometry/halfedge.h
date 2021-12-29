#ifndef HALFEDGE_H
#define HALFEDGE_H

class HalfEdge {
public:
    HalfEdge() {}

    int origin = -1;
    int twin = -1;
    int incidentFace = -1;
    int next = -1;
    int prev = -1;
    int id = -1;
};

#endif
