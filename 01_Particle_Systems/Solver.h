#ifndef SOLVER_H
#define SOLVER_H

#include "ParticleSystem.h"

using namespace std;

class Solver {
public:
    // Standard Euler step
    static void EulerStep(ParticleSystem& sys, double deltaT) {
        int dim = sys.getDim();
        vector<double> deriv(dim);
        vector<double> state(dim);

        sys.derivative(deriv);
        sys.getState(state);

        for (int i = 0; i < dim; ++i) {
            state[i] += deltaT * deriv[i];
        }

        sys.setState(state);
        sys.t += deltaT;
    }

    // Midpoint (RK2) step for greater stability
    static void MidpointStep(ParticleSystem& sys, double deltaT) {
        int dim = sys.getDim();
        vector<double> k1(dim), k2(dim);
        vector<double> initialState(dim), midState(dim);

        sys.getState(initialState);
        sys.derivative(k1);

        for (int i = 0; i < dim; ++i) {
            midState[i] = initialState[i] + 0.5 * deltaT * k1[i];
        }

        sys.setState(midState);
        sys.t += 0.5 * deltaT;

        sys.derivative(k2);

        for (int i = 0; i < dim; ++i) {
            initialState[i] += deltaT * k2[i];
        }

        sys.setState(initialState);
        sys.t += 0.5 * deltaT; // completing full step
    }
};

#endif