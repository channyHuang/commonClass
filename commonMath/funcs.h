#ifndef FUNCS_H
#define FUNCS_H

#include "vector3i.h"
#include "vector3.h"
#include "plane.h"
#include "box.h"

namespace MathFuncs {
    static constexpr float PI = 3.14159265f;
    static constexpr float LOWEPSILON = 1e-4f;
    static constexpr int MAX_LOD = 4;

    static constexpr float EPSILON = std::numeric_limits<float>::epsilon();
    static constexpr float POS_INFINITY = std::numeric_limits<float>::infinity();
    static constexpr float NEG_INFINITY = -std::numeric_limits<float>::infinity();

    template<class T>
    static T clamp(T v, T minn, T maxn) {
        if (v < minn) return minn;
        if (v > maxn) return maxn;
        return v;
    }

    template <typename T>
    static inline void sort_min_max(T& a, T& b) {
        if (a > b) {
            T temp = a;
            a = b;
            b = temp;
        }
    }

    template <typename T>
    static inline T Lerp(T a, T b, float alpha) {
        return (a + alpha * (b - a));
    }

    static int minAbsAxis(const Vector3& vec) {
        Vector3 vAbsVec = { std::fabs(vec.x), std::fabs(vec.y), std::fabs(vec.z) };
        return (vAbsVec.x < vAbsVec.y ? (vAbsVec.x < vAbsVec.z ? 0 : 2) : (vAbsVec.y < vAbsVec.z ? 1 : 2));
    }

    static float minDistToCube(const Vector3& vec, const Box& box) {
        Vector3 vDist;
        float res = -1;
        for (int i = 0; i < 3; i++) {
            vDist[i] = vec[i] <= box.vMin[i] ? box.vMin[i] - vec[i] : (vec[i] >= box.vMax[i] ? vec[i] - box.vMax[i] : std::fmax(box.vMin[i] - vec[i], vec[i] - box.vMax[i]));
            if (vDist[i] > 0) {
                if (res == -1) res = vDist[i];
                else res = std::fmin(res, vDist[i]);
            }
        }
        if (res > 0) return res;
        int dim = minAbsAxis(vDist);
        return vDist[dim];
    }

    static float sign(float v) {
        if (v == 0) return 0;
        return v > 0 ? 1.f : -1.f;
    }
    /*
    static float IntervalRandom (int fMin, int fMax, unsigned int uiSeed = 0)
    {
        return IntervalRandom(static_cast<float>(fMin), static_cast<float>(fMax), uiSeed);
    }*/

    static float IntervalRandom(float fMin, float fMax, unsigned int uiSeed = 0)
    {
        if (uiSeed > 0)
            srand(uiSeed);

        float dRatio = ((float)rand()) / ((float)(RAND_MAX));
        return (fMin + (fMax - fMin) * dRatio);
    }

    static float Clamp(float v, float fmin, float fmax) {
        return (v < fmin ? fmin : (v > fmax ? fmax : v));
    }

    static Vector3i vector3FloorOrCeil(const Vector3& pos, bool bfloor = true) {
        if (bfloor) {
            Vector3&& pos2floor = pos.getFloor();
            Vector3i&& vposi = Vector3i(static_cast<int>(pos2floor.x), static_cast<int>(pos2floor.y), static_cast<int>(pos2floor.z));
            return vposi;
        }
        //bceil
        Vector3&& pos2ceil = Vector3(pos);
        pos2ceil.ceil();
        Vector3i&& vposi = Vector3i(static_cast<int>(pos2ceil.x), static_cast<int>(pos2ceil.y), static_cast<int>(pos2ceil.z));
        return vposi;
    }

    static float get_vector3_angle(const Vector3& v1, const Vector3& v2) {
        if (v1 == Vector3(0) || v2 == Vector3(0)) return 0.f;
        float cos_theta = v1.dot(v2) / (v1.len() * v2.len());
        return acos(cos_theta); //[0,pi]
    }

    static float distancePoint2Plane(const Vector3& pos, const Plane& plane) {
        Vector3 dir = pos - plane.vPoint_;
        dir.normalize();
        float costheat = dir.dot(plane.vNormal_);
        float dist = (pos - plane.vPoint_).len() * std::fabs(costheat);
        return dist;
    }

    static Vector3 linePlaneIntersect(const Vector3& pos1, const Vector3& pos2, const Plane& plane) {
        float dist1 = distancePoint2Plane(pos1, plane);
        float dist2 = distancePoint2Plane(pos2, plane);
        bool side1 = ((pos1 - plane.vPoint_).dot(plane.vNormal_) >= 0);
        bool side2 = ((pos2 - plane.vPoint_).dot(plane.vNormal_) >= 0);
        Vector3 res;
        if (side1 == side2) {
            res = pos1 + (pos2 - pos1) * (dist1 / std::fabs(dist1 - dist2));
        }
        else {
            res = pos1 + (pos2 - pos1) * (dist1 / (dist1 + dist2));
        }
        return res;
    }
}

#endif