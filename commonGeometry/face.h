#ifndef FACE_H
#define FACE_H

#include <vector>

class Face {
public:
    Face() {}

    int outerComponent = -1;
    std::vector<int> innerComponents;
    int id = -1;
};

#endif
