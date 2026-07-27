#include <iostream>
#include <iomanip>
#include "ParticleSystem.h"
#include "Solver.h"

using namespace std;

int main() {
    ParticleSystem sys;

    // Create two particles connected by a spring
    // Particle 1: Fixed mass at origin
    Particle* p1 = new Particle(1.0, Vec3(0, 10, 0), Vec3(0, 0, 0));
    // Particle 2: Mass suspended below
    Particle* p2 = new Particle(1.0, Vec3(0, 5, 0), Vec3(0, 0, 0));

    sys.addParticle(p1);
    sys.addParticle(p2);

    // Apply global forces: Gravity (-9.8 m/s^2 on Y) and Viscous Drag
    sys.addForce(new GravityForce(sys.particles, Vec3(0, -9.81, 0)));
    sys.addForce(new DragForce(sys.particles, 0.1));

    // Connect p1 and p2 with a spring (ks = 50, kd = 0.5, restLength = 3.0)
    sys.addForce(new SpringForce(p1, p2, 50.0, 0.5, 3.0));

    cout << fixed << setprecision(3);
    cout << "Starting Particle Dynamics Simulation\n";
    cout << "--------------------------------------------------------\n";
    cout << "Time(s) | P1 Pos (X, Y, Z)        | P2 Pos (X, Y, Z)\n";
    cout << "--------------------------------------------------------\n";

    double dt = 0.01;
    int totalSteps = 200;

    for (int step = 0; step <= totalSteps; ++step) {
        if (step % 20 == 0) {
            cout << sys.t << "s  | "
                 << "(" << p1->x.x << ", " << p1->x.y << ", " << p1->x.z << ") | "
                 << "(" << p2->x.x << ", " << p2->x.y << ", " << p2->x.z << ")\n";
        }
        
        // Take an ODE integration step using Midpoint solver
        Solver::MidpointStep(sys, dt);
    }

    return 0;
}