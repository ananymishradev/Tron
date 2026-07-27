#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include <vector>
#include "Particle.h"
#include "Force.h"

using namespace std;

class ParticleSystem {
public:
    vector<Particle*> particles;
    vector<Force*> forces;
    double t;

    ParticleSystem() : t(0.0) {}

    ~ParticleSystem() {
        for (auto p : particles) delete p;
        for (auto f : forces) delete f;
    }

    void addParticle(Particle* p) { particles.push_back(p); }
    void addForce(Force* f) { forces.push_back(f); }

    int getDim() const { return 6 * particles.size(); }

    void getState(vector<double>& dst) const {
        dst.resize(getDim());
        int idx = 0;
        for (auto p : particles) {
            dst[idx++] = p->x.x; dst[idx++] = p->x.y; dst[idx++] = p->x.z;
            dst[idx++] = p->v.x; dst[idx++] = p->v.y; dst[idx++] = p->v.z;
        }
    }

    void setState(const vector<double>& src) {
        int idx = 0;
        for (auto p : particles) {
            p->x.x = src[idx++]; p->x.y = src[idx++]; p->x.z = src[idx++];
            p->v.x = src[idx++]; p->v.y = src[idx++]; p->v.z = src[idx++];
        }
    }

    void clearForces() {
        for (auto p : particles) p->clearForce();
    }

    void computeForces() {
        clearForces();
        for (auto f : forces) f->apply();
    }

    void derivative(vector<double>& dst) {
        computeForces();
        dst.resize(getDim());
        int idx = 0;
        for (auto p : particles) {
            // x_dot = v
            dst[idx++] = p->v.x;
            dst[idx++] = p->v.y;
            dst[idx++] = p->v.z;
            // v_dot = f / m
            dst[idx++] = p->f.x / p->m;
            dst[idx++] = p->f.y / p->m;
            dst[idx++] = p->f.z / p->m;
        }
    }
};

#endif