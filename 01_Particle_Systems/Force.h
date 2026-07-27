#ifndef FORCE_H
#define FORCE_H

#include "Particle.h"
#include <vector>

using namespace std;

class Force {
public:
    virtual ~Force() {}
    virtual void apply() = 0;
};

// Unary Force: Gravity
class GravityForce : public Force {
    vector<Particle*>& particles;
    Vec3 gravity;
public:
    GravityForce(vector<Particle*>& pList, Vec3 g) : particles(pList), gravity(g) {}

    void apply() override {
        for (auto p : particles) {
            p->f += p->m * gravity;
        }
    }
};

// Unary Force: Viscous Drag
class DragForce : public Force {
    vector<Particle*>& particles;
    double kdrag;
public:
    DragForce(vector<Particle*>& pList, double k) : particles(pList), kdrag(k) {}

    void apply() override {
        for (auto p : particles) {
            p->f -= kdrag * p->v;
        }
    }
};

// Binary Force: Hooke's Law Spring with Damping
class SpringForce : public Force {
    Particle* p1;
    Particle* p2;
    double ks; // Spring constant
    double kd; // Damping constant
    double r;  // Rest length
public:
    SpringForce(Particle* a, Particle* b, double ks, double kd, double restLength)
        : p1(a), p2(b), ks(ks), kd(kd), r(restLength) {}

    void apply() override {
        Vec3 l = p1->x - p2->x;
        Vec3 ldot = p1->v - p2->v;
        double len = l.length();

        if (len == 0.0) return;

        Vec3 lNorm = l / len;
        double springMag = - (ks * (len - r) + kd * ldot.dot(lNorm));
        Vec3 force = springMag * lNorm;

        p1->f += force;
        p2->f -= force;
    }
};

#endif