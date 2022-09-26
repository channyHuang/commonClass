#ifndef OCTREE_H
#define OCTREE_H

#include "vector3.h"
#include <unordered_map>

enum class AXIS3D : uint8_t {
    AXIS3D_NXNYNZ,
    AXIS3D_NXNYPZ,
    AXIS3D_NXPYNZ,
    AXIS3D_NXPYPZ,
    AXIS3D_PXNYNZ,
    AXIS3D_PXNYPZ,
    AXIS3D_PXPYNZ,
    AXIS3D_PXPYPZ,
    AXIS3D_COUNT
};

class Plane {
public:
    Plane() : normal(Vector3(0, 1, 0)) {}
    Plane(const Vector3& _point, const Vector3& _normal) : point(_point), normal(_normal) {}

private:
    Vector3 point;
    Vector3 normal;
};

class Frustum {
public:
    Frustum() {}
    void setPlane(int i, const Vector3& point, const Vector3& normal) {
        plane[i] = Plane(point, normal);
    }
    void setPlane(int i, const Plane& p) { plane[i] = p; }

private:
    Plane plane[6];
};

class AABB {
public:
    Vector3 vMin;
    Vector3 vMax;
    Vector3 vExtent;

    AABB() : vMin(Vector3(0)), vMax(Vector3(0)), vExtent(Vector3(0)) {}
    AABB(const Vector3 &_vExtent) : vExtent(_vExtent) {
        Vector3 vHalfExtent = _vExtent / 2.f;
        vMin = Vector3(0) - vHalfExtent;
        vMax = vHalfExtent;
    }
    Vector3 getExtent() {
        return vExtent;
    }
};

class Object {
public:
    AABB aabb;
    uint64_t id;
};

class OcTant {
public:
    OcTant() {}

    bool addObject(Object *object) {

    }

private:
    bool isLeaf = true;
    uint8_t childcount = 0;

    OcTant *parent = nullptr;
    OcTant *children[8] = {nullptr};
    AABB aabb;
};

class OcTree {
public:
    bool addObject(Object *object) {
        object->id = objectCount;
        mapTreeObjects[objectCount] = object;

        if (root == nullptr) {
            root = new OcTant();
        }
    }
    bool removeObject(uint64_t objectIdInOctree);
    bool moveObject(uint64_t objectIdInOctree);

private:
    OcTant *root = nullptr;
    uint64_t objectCount = 0;
    std::unordered_map<uint64_t, Object*> mapTreeObjects;
};

#endif
