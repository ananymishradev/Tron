#ifndef MATRIX3X3_H
#define MATRIX3X3_H

#include <iostream>
#include <iomanip>
#include "Vec3.h"

struct Matrix3x3 {
    double m[3][3] = {{0}};

    Matrix3x3() = default;

    double* operator[](int i) { return m[i]; }
    const double* operator[](int i) const { return m[i]; }

    static Matrix3x3 identity() {
        Matrix3x3 mat;
        mat.m[0][0] = mat.m[1][1] = mat.m[2][2] = 1.0;
        return mat;
    }

    static Matrix3x3 zeros() {
        return Matrix3x3();
    }

    Matrix3x3 transpose() const {
        Matrix3x3 res;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                res.m[i][j] = m[j][i];
        return res;
    }

    Matrix3x3 operator*(const Matrix3x3& o) const {
        Matrix3x3 res;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k)
                    res.m[i][j] += m[i][k] * o.m[k][j];
        return res;
    }

    Vec3 operator*(const Vec3& v) const {
        return Vec3(
            m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
            m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
            m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
        );
    }

    Matrix3x3 inverseDiagonal() const {
        Matrix3x3 res;
        res.m[0][0] = (m[0][0] != 0.0) ? 1.0 / m[0][0] : 0.0;
        res.m[1][1] = (m[1][1] != 0.0) ? 1.0 / m[1][1] : 0.0;
        res.m[2][2] = (m[2][2] != 0.0) ? 1.0 / m[2][2] : 0.0;
        return res;
    }

    double trace() const { return m[0][0] + m[1][1] + m[2][2]; }

    friend std::ostream& operator<<(std::ostream& os, const Matrix3x3& mat) {
        os << "┌" << std::setw(10) << mat.m[0][0] << " " << std::setw(10) << mat.m[0][1] << " " << std::setw(10) << mat.m[0][2] << "┐\n";
        os << "│" << std::setw(10) << mat.m[1][0] << " " << std::setw(10) << mat.m[1][1] << " " << std::setw(10) << mat.m[1][2] << "│\n";
        os << "└" << std::setw(10) << mat.m[2][0] << " " << std::setw(10) << mat.m[2][1] << " " << std::setw(10) << mat.m[2][2] << "┘";
        return os;
    }
};

#endif
