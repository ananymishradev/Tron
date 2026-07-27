#include <iostream>
#include <iomanip>
#include <random>
#include "ParticleSystem.h"
#include "Solver.h"

using namespace std;

double getRandom(double min, double max, mt19937& gen) {
    uniform_real_distribution<double> dist(min, max);
    return dist(gen);
}

int main() {
    random_device rd;
    mt19937 gen(rd());

    ParticleSystem sys;
    int numParticles = 5;

    cout << "Initializing " << numParticles << " randomized particles...\n";

    for (int i = 0; i < numParticles; ++i) {
        double m = getRandom(0.5, 2.0, gen);
        Vec3 pos(
            getRandom(-2.0, 2.0, gen),
            getRandom(5.0, 15.0, gen),
            getRandom(-2.0, 2.0, gen)
        );
        Vec3 vel(
            getRandom(-1.0, 1.0, gen),
            getRandom(-1.0, 1.0, gen),
            getRandom(-1.0, 1.0, gen)
        );

        sys.addParticle(new Particle(m, pos, vel));
    }

    sys.addForce(new GravityForce(sys.particles, Vec3(0, -9.81, 0)));
    sys.addForce(new DragForce(sys.particles, 0.15));

    for (size_t i = 0; i < sys.particles.size() - 1; ++i) {
        double ks = getRandom(30.0, 80.0, gen);
        double kd = getRandom(0.2, 1.0, gen);
        double restLen = getRandom(1.5, 3.0, gen);

        sys.addForce(new SpringForce(sys.particles[i], sys.particles[i+1], ks, kd, restLen));
    }

    double dt = 0.01;
    int totalSteps = 200;

    cout << "\n=========================================================================================================\n";
    cout << fixed << setprecision(3);
    cout << left
         << setw(8)  << "Time(s)"
         << setw(8)  << "P_ID"
         << setw(8)  << "Mass"
         << setw(26) << "Pos (X, Y, Z)"
         << setw(26) << "Vel (X, Y, Z)"
         << setw(14) << "Speed (|v|)"
         << setw(14) << "Force (|f|)"
         << setw(14) << "Kinetic (J)"
         << setw(14) << "Pot. (J)" << "\n";
    cout << "=========================================================================================================\n";

    for (int step = 0; step <= totalSteps; ++step) {
        if (step % 20 == 0) {
            double totalSystemKE = 0.0;
            double totalSystemPE = 0.0;

            sys.computeForces();

            for (size_t i = 0; i < sys.particles.size(); ++i) {
                Particle* p = sys.particles[i];
                double speed = p->v.length();
                double forceMag = p->f.length();

                double ke = 0.5 * p->m * speed * speed;
                double pe = p->m * 9.81 * p->x.y;

                totalSystemKE += ke;
                totalSystemPE += pe;

                cout << setw(8)  << sys.t
                     << setw(8)  << i
                     << setw(8)  << p->m
                     << "(" << setw(6) << p->x.x << "," << setw(6) << p->x.y << "," << setw(6) << p->x.z << ")  "
                     << "(" << setw(6) << p->v.x << "," << setw(6) << p->v.y << "," << setw(6) << p->v.z << ")  "
                     << setw(14) << speed
                     << setw(14) << forceMag
                     << setw(14) << ke
                     << setw(14) << pe << "\n";
            }
            cout << "--- Total System Mechanical Energy: " << (totalSystemKE + totalSystemPE) << " J ---\n";
            cout << "---------------------------------------------------------------------------------------------------------\n";
        }

        Solver::MidpointStep(sys, dt);
    }

    return 0;
}
