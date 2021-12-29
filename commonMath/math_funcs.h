#pragma once

#include "vector3.h"
#include "box.h"

class Math {
public:
	Math() {} // useless to instance

    template<class T>
    static T clamp(T v, T minn, T maxn) {
        if (v < minn) return minn;
        if (v > maxn) return maxn;
        return v;
    }

    template <typename T>
    static inline void sort_min_max(T &a, T &b) {
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

    static int minAbsAxis(const Vector3 &vec) {
        Vector3 vAbsVec = { std::fabs(vec.x), std::fabs(vec.y), std::fabs(vec.z) };
        return (vAbsVec.x < vAbsVec.y ? (vAbsVec.x < vAbsVec.z ? 0 : 2) : (vAbsVec.y < vAbsVec.z ? 1 : 2));
    }

    static float minDistToCube(Vector3 &vec, Box &box) {
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

        float dRatio = ((float)rand())/((float)(RAND_MAX));
        return (fMin + (fMax-fMin)*dRatio);
    }

    static float Clamp(float v, float fmin, float fmax) {
        return (v < fmin ? fmin : (v > fmax ? fmax : v));
    }

    static constexpr float PI = 3.14159265f;
    static constexpr float LOWEPSILON = 1e-4f;
    static constexpr int MAX_LOD = 4;

    static constexpr float EPSILON = std::numeric_limits<float>::epsilon();
    static constexpr float POS_INFINITY				= std::numeric_limits<float>::infinity();
    static constexpr float NEG_INFINITY				= -std::numeric_limits<float>::infinity();
};
