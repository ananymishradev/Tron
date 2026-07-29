#ifndef QUATERNION_H
#define QUATERNION_H

#include <cmath>
#include <iostream>
#include "Vec3.h"
#include "Matrix3x3.h"

struct Quaternion {
    double r = 1.0, i = 0.0, j = 0.0, k = 0.0;

    Quaternion() = default;
    Quaternion(double r, double i, double j, double k) : r(r), i(i), j(j), k(k) {}

    double normSq() const { return r * r + i * i + j * j + k * k; }
    double norm() const { return std::sqrt(normSq()); }

    void normalize() {
        double n = norm();
        if (n > 1e-12) { r /= n; i /= n; j /= n; k /= n; }
        else { r = 1.0; i = j = k = 0.0; }
    }

    Quaternion normalized() const {
        Quaternion q = *this;
        q.normalize();
        return q;
    }

    Quaternion conjugate() const { return {r, -i, -j, -k}; }

    Quaternion operator*(const Quaternion& q) const {
        return Quaternion(
            r * q.r - i * q.i - j * q.j - k * q.k,
            r * q.i + i * q.r + j * q.k - k * q.j,
            r * q.j - i * q.k + j * q.r + k * q.i,
            r * q.k + i * q.j - j * q.i + k * q.r
        );
    }

    Quaternion operator*(double s) const {
        return {r * s, i * s, j * s, k * s};
    }

    Quaternion operator+(const Quaternion& q) const {
        return {r + q.r, i + q.i, j + q.j, k + q.k};
    }

    Matrix3x3 toRotationMatrix() const {
        Matrix3x3 R;
        double ri = 2.0 * r * i, rj = 2.0 * r * j, rk = 2.0 * r * k;
        double ii = 2.0 * i * i, jj = 2.0 * j * j, kk = 2.0 * k * k;
        double ij = 2.0 * i * j, ik = 2.0 * i * k, jk = 2.0 * j * k;

        R.m[0][0] = 1.0 - jj - kk;
        R.m[0][1] = ij - rk;
        R.m[0][2] = ik + rj;
        R.m[1][0] = ij + rk;
        R.m[1][1] = 1.0 - ii - kk;
        R.m[1][2] = jk - ri;
        R.m[2][0] = ik - rj;
        R.m[2][1] = jk + ri;
        R.m[2][2] = 1.0 - ii - jj;
        return R;
    }

    static Quaternion fromAxisAngle(const Vec3& axis, double angle) {
        double half = angle * 0.5;
        double s = std::sin(half);
        Vec3 a = axis.normalized();
        return {std::cos(half), a.x * s, a.y * s, a.z * s};
    }

    friend std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
        os << "[" << q.r << ", " << q.i << ", " << q.j << ", " << q.k << "]";
        return os;
    }
};

#endif
