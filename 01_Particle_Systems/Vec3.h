#ifndef VEC3_H
#define VEC3_H

#include <iostream>
#include <cmath>

using namespace std;

struct Vec3 {
    double x, y, z;

    Vec3(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& v) const { return Vec3(x + v.x, y + v.y, z + v.z); }
    Vec3 operator-(const Vec3& v) const { return Vec3(x - v.x, y - v.y, z - v.z); }
    Vec3 operator*(double s) const { return Vec3(x * s, y * s, z * s); }
    Vec3 operator/(double s) const { return Vec3(x / s, y / s, z / s); }

    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3& operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }

    double length() const { return sqrt(x * x + y * y + z * z); }
    double dot(const Vec3& v) const { return x * v.x + y * v.y + z * v.z; }

    Vec3 normalized() const {
        double len = length();
        return len > 0 ? (*this) / len : Vec3(0, 0, 0);
    }
};

inline Vec3 operator*(double s, const Vec3& v) { return v * s; }

#endif