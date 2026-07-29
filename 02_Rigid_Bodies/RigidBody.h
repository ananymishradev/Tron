#ifndef RIGID_BODY_H
#define RIGID_BODY_H

#include <string>
#include "Vec3.h"
#include "Matrix3x3.h"
#include "Quaternion.h"

struct RigidBody {
    int id = 0;
    std::string name;

    double mass = 1.0;
    Matrix3x3 Ibody;
    Matrix3x3 IbodyInv;

    Vec3 x;
    Quaternion q;
    Vec3 P;
    Vec3 L;

    Matrix3x3 R;
    Matrix3x3 Iinv;
    Vec3 v;
    Vec3 omega;

    Vec3 force;
    Vec3 torque;

    void setBoxInertia(double w, double h, double d) {
        Ibody = Matrix3x3::zeros();
        Ibody.m[0][0] = (1.0 / 12.0) * mass * (h * h + d * d);
        Ibody.m[1][1] = (1.0 / 12.0) * mass * (w * w + d * d);
        Ibody.m[2][2] = (1.0 / 12.0) * mass * (w * w + h * h);
        IbodyInv = Ibody.inverseDiagonal();
    }

    void setSphereInertia(double radius) {
        double I = 0.4 * mass * radius * radius;
        Ibody = Matrix3x3::zeros();
        Ibody.m[0][0] = Ibody.m[1][1] = Ibody.m[2][2] = I;
        IbodyInv = Ibody.inverseDiagonal();
    }

    void computeAuxiliary() {
        q.normalize();
        R = q.toRotationMatrix();
        v = P / mass;
        Iinv = R * IbodyInv * R.transpose();
        omega = Iinv * L;
    }

    void applyForceAtPoint(const Vec3& F, const Vec3& p) {
        force += F;
        torque += (p - x).cross(F);
    }

    void applyForceAtCOM(const Vec3& F) {
        force += F;
    }

    void applyTorque(const Vec3& tau) {
        torque += tau;
    }

    void resetAccumulators() {
        force = Vec3(0, 0, 0);
        torque = Vec3(0, 0, 0);
    }

    double computeKineticEnergy() const {
        return 0.5 * mass * v.dot(v) + 0.5 * omega.dot(L);
    }

    double computePotentialEnergy(double g = 9.81) const {
        return mass * g * x.y;
    }

    double computeTotalEnergy(double g = 9.81) const {
        return computeKineticEnergy() + computePotentialEnergy(g);
    }
};

#endif
