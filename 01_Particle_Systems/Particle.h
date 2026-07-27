#ifndef PARTICLE_H
#define PARTICLE_H

#include "Vec3.h"

using namespace std;

struct Particle {
    double m;   // Mass
    Vec3 x;     // Position vector
    Vec3 v;     // Velocity vector
    Vec3 f;     // Force accumulator

    Particle(double mass, Vec3 pos, Vec3 vel = Vec3())
        : m(mass), x(pos), v(vel), f(Vec3(0, 0, 0)) {}

    void clearForce() {
        f = Vec3(0, 0, 0);
    }
};

#endif