# Physically Based Modeling

C++ implementations based on the CMU SIGGRAPH course notes on **Physically Based Modeling: Principles and Practice** by Andrew Witkin and David Baraff.

* Course Link: [CMU SIGGRAPH Course Notes](https://www.cs.cmu.edu/~baraff/sigcourse/)

---

## Directory Navigation

* [`01_Particle_Systems/`](./01_Particle_Systems/) — Particle dynamics, phase space formulation, forces, and collisions.
* [`02_Rigid_Bodies/`](./02_Rigid_Bodies/) — 3D rigid body kinematics, quaternions, mass properties, and forces/torques.
* [`03_Implicit_Methods/`](./03_Implicit_Methods/) — Implicit integration (Backward Euler) for stiff dynamic systems.
* [`04_Constrained_Dynamics/`](./04_Constrained_Dynamics/) — Equality constraints, Lagrange multipliers, LCP, and contact mechanics.

---

## Quick Start

```bash
g++ -o output main.cpp -std=c++17
```
