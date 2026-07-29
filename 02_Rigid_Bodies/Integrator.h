#ifndef INTEGRATOR_H
#define INTEGRATOR_H

#include <string>
#include "RigidBody.h"

constexpr int STATE_SIZE = 13;

// ============================================================================
// State Packing
// ============================================================================
inline void stateToArray(const RigidBody& rb, double y[]) {
    y[0]  = rb.x.x; y[1]  = rb.x.y; y[2]  = rb.x.z;
    y[3]  = rb.q.r; y[4]  = rb.q.i; y[5]  = rb.q.j; y[6]  = rb.q.k;
    y[7]  = rb.P.x; y[8]  = rb.P.y; y[9]  = rb.P.z;
    y[10] = rb.L.x; y[11] = rb.L.y; y[12] = rb.L.z;
}

inline void arrayToState(RigidBody& rb, const double y[]) {
    rb.x = Vec3(y[0], y[1], y[2]);
    rb.q = Quaternion(y[3], y[4], y[5], y[6]);
    rb.P = Vec3(y[7], y[8], y[9]);
    rb.L = Vec3(y[10], y[11], y[12]);
    rb.computeAuxiliary();
}

inline void computeDerivatives(RigidBody& rb, const double y[], double ydot[]) {
    arrayToState(rb, y);

    ydot[0] = rb.v.x;
    ydot[1] = rb.v.y;
    ydot[2] = rb.v.z;

    Quaternion w_quat(0, rb.omega.x, rb.omega.y, rb.omega.z);
    Quaternion dqdt = w_quat * rb.q;
    ydot[3] = 0.5 * dqdt.r;
    ydot[4] = 0.5 * dqdt.i;
    ydot[5] = 0.5 * dqdt.j;
    ydot[6] = 0.5 * dqdt.k;

    ydot[7] = rb.force.x;
    ydot[8] = rb.force.y;
    ydot[9] = rb.force.z;

    ydot[10] = rb.torque.x;
    ydot[11] = rb.torque.y;
    ydot[12] = rb.torque.z;
}

// ============================================================================
// Integrator Types
// ============================================================================
enum class IntegratorType {
    EXPLICIT_EULER,
    RK2_MIDPOINT,
    RK4,
    SYMPLECTIC_EULER
};

inline std::string integratorName(IntegratorType type) {
    switch (type) {
        case IntegratorType::EXPLICIT_EULER: return "Explicit Euler";
        case IntegratorType::RK2_MIDPOINT:   return "RK2 Midpoint";
        case IntegratorType::RK4:            return "RK4 (Classical)";
        case IntegratorType::SYMPLECTIC_EULER: return "Symplectic Euler";
    }
    return "Unknown";
}

// ============================================================================
// Step Functions
// ============================================================================
inline void stepEuler(RigidBody& rb, double dt) {
    double y0[STATE_SIZE], ydot[STATE_SIZE], y1[STATE_SIZE];
    stateToArray(rb, y0);
    computeDerivatives(rb, y0, ydot);
    for (int i = 0; i < STATE_SIZE; ++i)
        y1[i] = y0[i] + dt * ydot[i];
    arrayToState(rb, y1);
}

inline void stepRK2(RigidBody& rb, double dt) {
    double y0[STATE_SIZE], k1[STATE_SIZE], y_mid[STATE_SIZE], k2[STATE_SIZE], y1[STATE_SIZE];
    stateToArray(rb, y0);
    computeDerivatives(rb, y0, k1);
    for (int i = 0; i < STATE_SIZE; ++i)
        y_mid[i] = y0[i] + 0.5 * dt * k1[i];
    computeDerivatives(rb, y_mid, k2);
    for (int i = 0; i < STATE_SIZE; ++i)
        y1[i] = y0[i] + dt * k2[i];
    arrayToState(rb, y1);
}

inline void stepRK4(RigidBody& rb, double dt) {
    double y[STATE_SIZE], k1[STATE_SIZE], k2[STATE_SIZE], k3[STATE_SIZE], k4[STATE_SIZE], tmp[STATE_SIZE];
    stateToArray(rb, y);

    computeDerivatives(rb, y, k1);
    for (int i = 0; i < STATE_SIZE; ++i) tmp[i] = y[i] + 0.5 * dt * k1[i];
    computeDerivatives(rb, tmp, k2);
    for (int i = 0; i < STATE_SIZE; ++i) tmp[i] = y[i] + 0.5 * dt * k2[i];
    computeDerivatives(rb, tmp, k3);
    for (int i = 0; i < STATE_SIZE; ++i) tmp[i] = y[i] + dt * k3[i];
    computeDerivatives(rb, tmp, k4);

    double y1[STATE_SIZE];
    for (int i = 0; i < STATE_SIZE; ++i)
        y1[i] = y[i] + (dt / 6.0) * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);
    arrayToState(rb, y1);
}

inline void stepSymplecticEuler(RigidBody& rb, double dt) {
    rb.P += rb.force * dt;
    rb.L += rb.torque * dt;

    rb.computeAuxiliary();

    rb.x += rb.v * dt;

    Quaternion w_quat(0, rb.omega.x, rb.omega.y, rb.omega.z);
    Quaternion dqdt = w_quat * rb.q;
    rb.q.r += 0.5 * dqdt.r * dt;
    rb.q.i += 0.5 * dqdt.i * dt;
    rb.q.j += 0.5 * dqdt.j * dt;
    rb.q.k += 0.5 * dqdt.k * dt;
    rb.q.normalize();

    rb.computeAuxiliary();
}

inline void stepIntegrator(RigidBody& rb, double dt, IntegratorType type) {
    switch (type) {
        case IntegratorType::EXPLICIT_EULER:
            stepEuler(rb, dt);
            break;
        case IntegratorType::RK2_MIDPOINT:
            stepRK2(rb, dt);
            break;
        case IntegratorType::RK4:
            stepRK4(rb, dt);
            break;
        case IntegratorType::SYMPLECTIC_EULER:
            stepSymplecticEuler(rb, dt);
            break;
    }
}

#endif
