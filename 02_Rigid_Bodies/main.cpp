#include <iostream>
#include <iomanip>
#include "Simulation.h"

int main() {
    std::cout << std::setprecision(4) << std::fixed;

    {
        SimulationConfig cfg;
        cfg.dt = 0.02;
        cfg.totalSteps = 20;
        cfg.numBodies = 3;
        cfg.gravity = 9.81;
        cfg.enableGround = true;
        cfg.enableRandomForces = true;
        cfg.integrator = IntegratorType::RK4;
        cfg.seed = 42;

        SimulationEngine sim(cfg);
        sim.run();
    }

    {
        SimulationConfig cfg;
        cfg.dt = 0.01;
        cfg.totalSteps = 20;
        cfg.numBodies = 1;
        cfg.gravity = 9.81;
        cfg.enableGround = true;
        cfg.enableRandomForces = true;
        cfg.integrator = IntegratorType::SYMPLECTIC_EULER;
        cfg.seed = 99;

        SimulationEngine sim(cfg);
        sim.run();
    }

    return 0;
}
