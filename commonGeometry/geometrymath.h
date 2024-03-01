#ifndef GEOMETRYMATH_H
#define GEOMETRYMATH_H

#include <vector>
#include <cmath>

#include "commonMath/vector2.h"
#include "commonMath/vector3.h"
#include "commonMath/vector3i.h"
#include "commonMath/funcs.h"
#include "commonMath/boxi.h"

namespace GeometryMath {

static std::vector<Vector3> vDir8Corner = {
    {1, 1, 1}, {1, -1, 1}, {-1, -1, 1}, {-1, 1, 1},
    {1, 1, -1}, {1, -1, -1}, {-1, -1, -1}, {-1, 1, -1}
};
static std::vector<Vector3> vCubeVer = {
    {0,0,0}, {1,0,0}, {1,1,0}, {0,1,0},
    {0,0,1}, {1,0,1}, {1,1,1}, {0,1,1}
};
static std::vector<Vector3> vAxes = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };

inline float volumn(const Vector3 &vec) {
    return vec.x * vec.y * vec.z;
}

inline Vector3 vertexInterp(const Vector3 &p1, const Vector3 &p2, float weightP1, float weightP2) {
    if (std::abs(weightP2 - weightP1) < MathFuncs::EPSILON) {
        return (p1 + p2) * 0.5f;
    }
    return (p1 + (p2 - p1) * (-weightP1 / (weightP2 - weightP1)));
}

static Vector3i clamp_to(Vector3 target, const Vector3& min, const Vector3& max) {
    if (target.x < min.x) target.x = min.x;
    if (target.y < min.y) target.y = min.y;
    if (target.z < min.z) target.z = min.z;

    if (target.x >= max.x) target.x = max.x - 1;
    if (target.y >= max.y) target.y = max.y - 1;
    if (target.z >= max.z) target.z = max.z - 1;

    Vector3i res = Vector3i((int)target.x, (int)target.y, (int)target.z);
    return res;
}

inline int maxAbsAxis(const Vector3 &vec) {
    Vector3 vAbsVec = { std::fabs(vec.x), std::fabs(vec.y), std::fabs(vec.z) };
    return (vAbsVec.x > vAbsVec.y ? (vAbsVec.x > vAbsVec.z ? 0 : 2) : (vAbsVec.y > vAbsVec.z ? 1 : 2));
}

inline int minAbsAxis(const Vector3 &vec) {
    Vector3 vAbsVec = { std::fabs(vec.x), std::fabs(vec.y), std::fabs(vec.z) };
    return (vAbsVec.x < vAbsVec.y ? (vAbsVec.x < vAbsVec.z ? 0 : 2) : (vAbsVec.y < vAbsVec.z ? 1 : 2));
}

inline float minDistToCube(const Vector3 &vec, const Box &box) {
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

inline float sign(const float& val) {
    return (val < 0 ? -1.0f : val > 0 ? +1.0f : 0);
}

inline bool isFinite(const float t) {
    return std::isfinite(t) && !std::isnan(t);
}

inline bool isFinite(const Vector3 &v) {
    return isFinite(v.x) && isFinite(v.y) && isFinite(v.z);
}

template <typename T>
inline void sort_min_max(T& a, T& b) {
    if (a > b) {
        T tmp = a;
        a = b;
        b = tmp;
    }
}

static void sort_min_max(Vector3& a, Vector3& b) {
    sort_min_max(a.x, b.x);
    sort_min_max(a.y, b.y);
    sort_min_max(a.z, b.z);
}

inline bool equalToZero(float v) {
    return (v <= MathFuncs::EPSILON || -v >= MathFuncs::EPSILON);
}

//2d point a - b - c is anti cross wise
//cross > 0 - anti cross wise
static int TriangleIsCCW(const Vector2 &a, const Vector2 &b, const Vector2 &c) {
    float cross = ((a.x - c.x) * (b.y - c.y)) - ((a.y - c.y) * (b.x - c.x));
    return (cross == 0 ? 0 : (cross > 0 ? 1 : -1));
}

static int PointInConvexPolygon(const Vector2 &p, int n, const std::vector<Vector2>& v)
{
    if (n < 3) return 0;

    int low = 0, high = n;
    do {
        int mid = (low + high) / 2;
        int bCCW = TriangleIsCCW(v[0], v[mid], p);
        if (bCCW < 0)
            low = mid;
        else if (bCCW > 0)
            high = mid;
        else {
            if (mid == 1 || mid == n - 1) return 0;
            if (p.x > std::min(v[0].x, v[mid].x) && p.x < std::max(v[0].x, v[mid].x)
                && p.y > std::min(v[0].x, v[mid].x) && p.y < std::max(v[0].x, v[mid].x)) return 1;
            return -1;
        }
    } while (low + 1 < high);

    // If point outside last (or first) edge, then it is not inside the n-gon
    if (low == 0 || high == n) return 0;

    // p is inside the polygon if it is left of
    // the directed edge from v[low] to v[high]
    return (TriangleIsCCW(v[low], v[high], p) < 0 ? 1 : -1);
}

inline bool lineIntersection(const Vector2 &p, const Vector2 &r,
    const Vector2 &q, const Vector2 &s,
    Vector2 *intersect) {
    float cross = r.x*s.y - r.y*s.x;

    float eps = 1e-9f;
    if (fabsf(cross) < eps) {
        // parallel case
        return false;
    }

    float vx = q.x - p.x;
    float vy = q.y - p.y;
    float t = (vx*s.y - vy * s.x) / cross;

    intersect->x = p.x + t * r.x;
    intersect->y = p.y + t * r.y;

    return true;
}

static Vector2 randomPoint(const Boxi &extents) {
    float px = MathFuncs::IntervalRandom(extents.vMin.x, extents.vMax.x);
    float py = MathFuncs::IntervalRandom(extents.vMin.y, extents.vMax.y);

    return Vector2(px, py);
}

static Vector2 randomPoint(const Box &extents) {
    float px = MathFuncs::IntervalRandom(extents.vMin.x, extents.vMax.x);
    float py = MathFuncs::IntervalRandom(extents.vMin.y, extents.vMax.y);

    return Vector2(px, py);
}

static Vector3 randomPoint3d(const Box &extents) {
    float px = MathFuncs::IntervalRandom(extents.vMin.x, extents.vMax.x);
    float py = MathFuncs::IntervalRandom(extents.vMin.y, extents.vMax.y);
    float pz = MathFuncs::IntervalRandom(extents.vMin.z, extents.vMax.z);
    return Vector3(px, py, pz);
}

static Vector2 randomDirection() {
    float angle = MathFuncs::IntervalRandom(0.0f, 2 * MathFuncs::PI);
    return Vector2(sin(angle), cos(angle));
}

inline bool lineSegmentIntersection(const Vector2 &A, const Vector2 &B,
    const Vector2 &C, const Vector2 &D) {
    bool c1 = (D.y - A.y)*(C.x - A.x) > (C.y - A.y)*(D.x - A.x);
    bool c2 = (D.y - B.y)*(C.x - B.x) > (C.y - B.y)*(D.x - B.x);
    bool c3 = (C.y - A.y)*(B.x - A.x) > (B.y - A.y)*(C.x - A.x);
    bool c4 = (D.y - A.y)*(B.x - A.x) > (B.y - A.y)*(D.x - A.x);

    return (c1 != c2) && (c3 != c4);
}


// Given segment ab and point c, computes closest point d on ab.
// Also returns t for the position of d, d(t) = a + t*(b - a)
inline float distancePointSegment(const Vector3 &c, const Vector3 &a, const Vector3 &b)
{
    Vector3 ab = b - a;
    // Project c onto ab, computing parameterized position d(t) = a + t*(b ?a)
    float t = (c - a).dot(ab) / ab.dot(ab);
    // If outside segment, clamp t (and therefore d) to the closest endpoint
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    // Compute projected position from the clamped t
    Vector3 vClosePoint = a + t * ab;
    return vClosePoint.distanceTo(c);
}

inline float distancePointSegment(const Vector2 &c, const Vector2 &a, const Vector2 &b)
{
    Vector2 ab = b - a;
    // Project c onto ab, computing parameterized position d(t) = a + t*(b ?a)
    float t = (c - a).dot(ab) / ab.dot(ab);
    // If outside segment, clamp t (and therefore d) to the closest endpoint
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    // Compute projected position from the clamped t
    Vector2 vClosePoint = a + t * ab;
    return vClosePoint.distance(c);
}

}

#endif
