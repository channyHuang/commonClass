#pragma once

#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <string>

#include "vector3.h"

class Vector3i
{
public:
    union {
        struct {
            int x, y, z;
        };
        int data[3];
    };

    const int &operator[](const int axis) const {
        return data[axis];
    }

    inline int &operator[](const int axis) {
        return data[axis];
    }

    Vector3i() : x(0), y(0), z(0) {}
    Vector3i(const int v) : x(v), y(v), z(v) {}
    Vector3i(const int _x, const int _y, const int _z) : x(_x), y(_y), z(_z) {}
    Vector3i(const Vector3i &v) : x(v.x), y(v.y), z(v.z) {}
    Vector3i(const Vector3 &v) {
        x = static_cast<int>(std::floor(v.x));
        y = static_cast<int>(std::floor(v.y));
        z = static_cast<int>(std::floor(v.z));
    }

    ~Vector3i() {}

    //void operator=(const Vector3i &v) { x = v.x; y = v.y; z = v.z; }
    Vector3i &operator=(const Vector3i &v) {
        x = v.x;
        y = v.y;
        z = v.z;
        return *this;
    }

    inline bool operator==(const Vector3i &v) const {
        return x == v.x && y == v.y && z == v.z;
    }

    inline bool operator!=(const Vector3i &v) const {
        return x != v.x || y != v.y || z != v.z;
    }

    Vector3i operator+(const Vector3i &v) const {
        return Vector3i (x + v.x, y + v.y, z + v.z);
    }

    Vector3i &operator += (const Vector3i &v) {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    Vector3i operator-(const Vector3i &v) const {
        return Vector3i (x - v.x, y - v.y, z - v.z);
    }

    Vector3i &operator -= (const Vector3i &v) {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    Vector3i operator/(const Vector3i &v) {
        if (v.x == 0 || v.y == 0 || v.z == 0) return *this;
        return Vector3i (x / v.x, y / v.y, z / v.z);
    }

    Vector3i operator*(const Vector3i &v) const {
        return Vector3i (x * v.x, y * v.y, z * v.z);
    }

    Vector3i operator+(const int v) {
        return *this + Vector3i(v);
    }

    Vector3i operator-(const int v) {
        return *this - Vector3i(v);
    }

    Vector3i operator/(const int v) {
        return *this / Vector3i(v);
    }

    Vector3i operator*(const int v) const {
        return Vector3i(x * v, y * v, z * v);
    }

    float dot(const Vector3i &v) const{
        return (x * v.x + y * v.y + z * v.z);
    }

    Vector3i cross(const Vector3i &v) {
        return Vector3i(y * v.z - z * v.y, x * v.z - z * v.x, x * v.y - y * v.x);
    }

    int volumn() {
        return x * y * z;
    }

    float len() {
        return static_cast<float>(std::sqrt(x * x + y * y + z * z));
    }

    float lenSqr() {
        return static_cast<float>(x * x + y * y + z * z);
    }

    float distance(const Vector3i &v) {
        return static_cast<float>(std::sqrt((x - v.x) * (x - v.x) + (y - v.y) * (y - v.y) + (z - v.z) * (z - v.z)));
    }
    void normalize() {
        float length = this->len();
        if (length == 0) length = 1.0f;
        x /= length;
        y /= length;
        z /= length;
    }

    std::string toString() const {
        std::string str = "(" + std::to_string(x) + " " + std::to_string(y) + " " + std::to_string(z) + ")";
        return str;
    }

    inline unsigned int get_zxy_index(const Vector3i area_size) const {
        return y + area_size.y * (x + area_size.x * z); // ZXY
    }

    Vector3 to_vec3() const {
        return Vector3(x, y, z);
    }

    static void sort_min_max(Vector3i &a, Vector3i &b) {
        if (a.x > b.x) {
            int tmp = a.x;
            a.x = b.x;
            b.x = tmp;
        }
        if (a.y > b.y) {
            int tmp = a.y;
            a.y = b.y;
            b.y = tmp;
        }
        if (a.z > b.z) {
            int tmp = a.z;
            a.z = b.z;
            b.z = tmp;
        }
    }

    // Clamps between min and max, where max is excluded
    void clamp_to(const Vector3i min, const Vector3i max) {
        if (x < min.x) {
            x = min.x;
        }
        if (y < min.y) {
            y = min.y;
        }
        if (z < min.z) {
            z = min.z;
        }

        if (x >= max.x) {
            x = max.x - 1;
        }
        if (y >= max.y) {
            y = max.y - 1;
        }
        if (z >= max.z) {
            z = max.z - 1;
        }
    }
};
/*
inline Vector3i operator+(const Vector3i &a, const Vector3i &b) {
    return Vector3i(a.x + b.x, a.y + b.y, a.z + b.z);
}

inline Vector3i operator-(const Vector3i &a, const Vector3i &b) {
    return Vector3i(a.x - b.x, a.y - b.y, a.z - b.z);
}
*/
inline Vector3i operator>>(const Vector3i &a, int b) {
    return Vector3i(a.x >> b, a.y >> b, a.z >> b);
}

inline Vector3i operator<<(const Vector3i &a, int b) {
    return Vector3i(a.x << b, a.y << b, a.z << b);
}

inline Vector3i operator*(const int p_scalar, const Vector3i &p_vec) {
    return p_vec * p_scalar;
}

static inline uint32_t hash_djb2_one_32(uint32_t in, uint32_t prev = 5381) {
    return ((prev << 5) + prev) + in;
}

struct Vector3iHasher {
    size_t operator()(const Vector3i &vk) const {
        uint32_t h = hash_djb2_one_32(vk.x);
        h = hash_djb2_one_32(vk.y, h);
        h = hash_djb2_one_32(vk.z, h);
        return h;
    }
};

struct Vector3iHash {
    size_t operator()(const Vector3i &vk) const {
        return std::hash<std::string>()(std::to_string(vk.x) + "," + std::to_string(vk.y) + "," + std::to_string(vk.z));
    }
};
