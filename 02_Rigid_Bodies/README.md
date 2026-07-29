# Unconstrained Rigid Body Dynamics

> *"The rigid body is the simplest nontrivial dynamical system — six degrees of freedom, spinning and tumbling through space, governed by nothing but Newton and Euler."*

This is a full 3D rigid body dynamics simulator based on David Baraff's SIGGRAPH course notes (`notesd1.pdf`). It simulates the unconstrained motion of multiple rigid bodies under gravity, external impulses, and ground collisions — all integrated with a choice of four numerical schemes.

Each body has **13 state variables**: position $\mathbf{x}(t)$, orientation quaternion $\mathbf{q}(t)$, linear momentum $\mathbf{P}(t)$, and angular momentum $\mathbf{L}(t)$. The simulation evolves them through time, computing derived quantities like the rotation matrix $\mathbf{R}(t)$, the world-space inverse inertia tensor $\mathbf{I}^{-1}(t)$, linear velocity $\mathbf{v}(t)$, and angular velocity $\boldsymbol{\omega}(t)$ at every step.

---

## The Physics

Unlike point masses (which only translate), rigid bodies also **rotate**. A force applied anywhere other than the center of mass produces a torque, which changes the body's angular momentum. This coupling between translation and rotation is what makes rigid body dynamics interesting.

### State Space

Every rigid body has a 13-dimensional state vector:

$$\mathbf{Y}(t) = \begin{pmatrix} \mathbf{x}(t) \\ \mathbf{q}(t) \\ \mathbf{P}(t) \\ \mathbf{L}(t) \end{pmatrix} \in \mathbb{R}^{13}$$

| Component | Symbol | Size | Meaning |
|-----------|--------|------|---------|
| Position | $\mathbf{x}$ | 3 | Center of mass location in world space |
| Orientation | $\mathbf{q}$ | 4 | Unit quaternion representing rotation from body to world |
| Linear momentum | $\mathbf{P}$ | 3 | $\mathbf{P} = M\mathbf{v}$ |
| Angular momentum | $\mathbf{L}$ | 3 | $\mathbf{L} = \mathbf{I}\boldsymbol{\omega}$ |

For $N$ bodies, the full simulation state lives in $\mathbb{R}^{13N}$.

### Equations of Motion

The time derivative of the state vector is:

$$\frac{d}{dt} \mathbf{Y}(t) = \begin{pmatrix} \dot{\mathbf{x}} \\ \dot{\mathbf{q}} \\ \dot{\mathbf{P}} \\ \dot{\mathbf{L}} \end{pmatrix} = \begin{pmatrix} \mathbf{v} \\ \frac{1}{2}\boldsymbol{\omega}\,\mathbf{q} \\ \mathbf{F} \\ \boldsymbol{\tau} \end{pmatrix}$$

Each row comes from a different physical law:

1. **Translational kinematics**: $\dot{\mathbf{x}} = \mathbf{v}$ — position changes at the current velocity.
2. **Rotational kinematics**: $\dot{\mathbf{q}} = \frac{1}{2}\boldsymbol{\omega}\,\mathbf{q}$ — quaternion orientation evolves via a quaternion product with angular velocity (Baraff Section 4).
3. **Newton's second law**: $\dot{\mathbf{P}} = \mathbf{F}$ — linear momentum changes due to net force.
4. **Euler's equation**: $\dot{\mathbf{L}} = \boldsymbol{\tau}$ — angular momentum changes due to net torque.

### Derived Quantities

At every step, a handful of auxiliary variables are computed from the state:

#### Rotation Matrix $\mathbf{R}(t)$

The quaternion $\mathbf{q} = (r, i, j, k)$ converts to a $3\times3$ rotation matrix:

$$\mathbf{R} = \begin{pmatrix}
1 - 2(j^2 + k^2) & 2(ij - rk) & 2(ik + rj) \\
2(ij + rk) & 1 - 2(i^2 + k^2) & 2(jk - ri) \\
2(ik - rj) & 2(jk + ri) & 1 - 2(i^2 + j^2)
\end{pmatrix}$$

This matrix rotates vectors from **body space** to **world space**. A point $\mathbf{p}_0$ fixed in the body's local frame maps to the world as:

$$\mathbf{p}(t) = \mathbf{R}(t)\,\mathbf{p}_0 + \mathbf{x}(t)$$

#### Inertia Tensor Transformation

The inertia tensor is constant in body space ($\mathbf{I}_{\text{body}}$) but changes in world space as the body rotates:

$$\mathbf{I}^{-1}(t) = \mathbf{R}(t)\,\mathbf{I}_{\text{body}}^{-1}\,\mathbf{R}(t)^T$$

This is the **parallel axis theorem for rotation** — the same tensor, just seen from a different coordinate system.

#### Angular Velocity

Angular velocity is recovered from angular momentum via the inverse inertia tensor:

$$\boldsymbol{\omega}(t) = \mathbf{I}^{-1}(t)\,\mathbf{L}(t)$$

This is the rotational analogue of $\mathbf{v} = \mathbf{P} / M$. While $\mathbf{P}$ and $\mathbf{v}$ always point in the same direction (for a scalar mass), $\mathbf{L}$ and $\boldsymbol{\omega}$ need not be parallel — that's where precession comes from.

### Forces and Torques

Forces are accumulated each step and applied to the state derivatives:

- **Gravity at COM**: $\mathbf{F} = M\mathbf{g}$ (applied at the center of mass, so no torque).
- **Off-center impulse**: A force $\mathbf{F}$ applied at point $\mathbf{p}$ generates torque:
  
  $$\boldsymbol{\tau} = (\mathbf{p} - \mathbf{x}) \times \mathbf{F}$$

  This is the rotational equivalent of a lever arm: push off-center and the body spins.

The net force $\mathbf{F}$ and net torque $\boldsymbol{\tau}$ feed directly into $\dot{\mathbf{P}}$ and $\dot{\mathbf{L}}$.

### Energy

**Kinetic energy** has translational and rotational components:

$$E_k = \frac{1}{2} M \mathbf{v} \cdot \mathbf{v} + \frac{1}{2} \boldsymbol{\omega} \cdot \mathbf{L}$$

The second term can also be written as $\frac{1}{2}\boldsymbol{\omega}^T \mathbf{I} \boldsymbol{\omega}$, but $\boldsymbol{\omega} \cdot \mathbf{L}$ is cheaper to compute since $\mathbf{L}$ is already in the state.

### Ground Collision

A simple penalty-based ground contact model:

1. If $\mathbf{x}_y < 0$, the body is pushed back to $\mathbf{x}_y = 0^+$ and the normal velocity is reversed with restitution $e$:

   $$j_n = -(1 + e) \, v_n \, M \quad \Longrightarrow \quad \mathbf{P}_y \mathrel{+}= j_n$$

2. Tangential friction damps horizontal and angular velocities proportional to a friction coefficient.

---

## Numerical Integration

The simulation provides four integrators. Each operates on the same 13-element state vector via the `stateToArray`/`arrayToState`/`computeDerivatives` interface.

### Integrators

| Integrator | Order | Stability | Energy Drift | Use Case |
|-----------|-------|-----------|-------------|----------|
| Explicit Euler | 1st | Poor | Grows over time | Reference only |
| RK2 Midpoint | 2nd | Good | Moderate | Quick tests |
| RK4 | 4th | Excellent | Very low | Production use |
| Symplectic Euler | 1st | Good (structure-preserving) | Bounded | Long-term mechanical systems |

#### Explicit Euler

$$\mathbf{Y}_{n+1} = \mathbf{Y}_n + \Delta t \, \dot{\mathbf{Y}}_n$$

The simplest possible integrator. First-order accurate ($O(\Delta t)$). Tends to inject energy into the system — angular momenta drift upward over time. Included for comparison.

#### RK2 Midpoint

$$\begin{aligned}
\mathbf{k}_1 &= \dot{\mathbf{Y}}(\mathbf{Y}_n) \\
\mathbf{k}_2 &= \dot{\mathbf{Y}}(\mathbf{Y}_n + \tfrac{1}{2}\Delta t \, \mathbf{k}_1) \\
\mathbf{Y}_{n+1} &= \mathbf{Y}_n + \Delta t \, \mathbf{k}_2
\end{aligned}$$

Second-order accurate ($O(\Delta t^2)$). Evaluates the derivative at the midpoint of the interval. Good balance of cost and accuracy for moderate time steps.

#### RK4 (Classical)

The workhorse:

$$\begin{aligned}
\mathbf{k}_1 &= \dot{\mathbf{Y}}(\mathbf{Y}_n) \\
\mathbf{k}_2 &= \dot{\mathbf{Y}}(\mathbf{Y}_n + \tfrac{1}{2}\Delta t \, \mathbf{k}_1) \\
\mathbf{k}_3 &= \dot{\mathbf{Y}}(\mathbf{Y}_n + \tfrac{1}{2}\Delta t \, \mathbf{k}_2) \\
\mathbf{k}_4 &= \dot{\mathbf{Y}}(\mathbf{Y}_n + \Delta t \, \mathbf{k}_3) \\
\mathbf{Y}_{n+1} &= \mathbf{Y}_n + \tfrac{\Delta t}{6}(\mathbf{k}_1 + 2\mathbf{k}_2 + 2\mathbf{k}_3 + \mathbf{k}_4)
\end{aligned}$$

Fourth-order accurate ($O(\Delta t^4)$). Four evaluations per step but far higher accuracy — allows larger time steps for a given error tolerance.

#### Symplectic Euler

A semi-implicit method: update momenta first, then use the new velocities to update positions:

$$\begin{aligned}
\mathbf{P}_{n+1} &= \mathbf{P}_n + \Delta t \, \mathbf{F}_n \\
\mathbf{L}_{n+1} &= \mathbf{L}_n + \Delta t \, \boldsymbol{\tau}_n \\
\mathbf{x}_{n+1} &= \mathbf{x}_n + \Delta t \, \mathbf{v}_{n+1} \\
\mathbf{q}_{n+1} &= \mathbf{q}_n + \tfrac{1}{2}\Delta t \, \boldsymbol{\omega}_{n+1}\mathbf{q}_n
\end{aligned}$$

This is **symplectic**: it preserves the geometric structure of Hamiltonian mechanics. Energy drift is bounded (it oscillates rather than monotonically growing), making it ideal for long-duration simulations where energy conservation matters more than per-step accuracy.

### ODE State Management

The integrators don't manipulate `RigidBody` fields directly. Instead, they use three bridging functions:

```
stateToArray(rb, y)       → packs 13 fields into a double[13]
computeDerivatives(rb, y, ydot) → evaluates dY/dt at state y
arrayToState(rb, y)       → unpacks 13 doubles back into the RigidBody
```

This decoupling means the integrators are **completely agnostic** to what the state represents — they just shuffle numbers around. Physics lives in `computeDerivatives`.

---

## Collision Model

The ground plane collision is a simplified impulse-based model, not a full constrained dynamics solver:

```cpp
// Normal impulse (restitution)
j_n = -(1 + e) * v_n / (1/M)
P_y += j_n

// Tangential friction (velocity damping)
v_x *= (1 - μ * 0.05)
v_z *= (1 - μ * 0.05)

// Angular friction
ω *= (1 - μ * 0.02)
```

This is deliberately minimal — it keeps bodies from falling through the floor without the complexity of a full LCP or penalty-based contact solver. Parameters:

| Parameter | Value | Meaning |
|-----------|-------|---------|
| $e$ (restitution) | 0.5 | Bounciness: 0 = inelastic, 1 = perfectly elastic |
| $\mu$ (friction) | 0.3 | Coulomb friction coefficient |

---

## The Code

```
02_Rigid_Bodies/
├── Vec3.h          — 3D vector arithmetic (dot, cross, normalize)
├── Matrix3x3.h     — 3×3 matrix (multiplication, transpose, diagonal inverse)
├── Quaternion.h    — Quaternion ops (product, normalize, rotation matrix, axis-angle)
├── RigidBody.h     — Rigid body struct (state, inertia, forces, energy)
├── Integrator.h    — State vector packing + Euler/RK2/RK4/Symplectic steps
├── Simulation.h    — Config, body generator, logging, Unicode table output, engine
├── main.cpp        — Entry point with two demo configurations
└── build/
    └── main.out
```

### Architecture

The design mirrors the particle system: math primitives at the bottom, physics objects in the middle, integration and simulation on top.

```
                    ┌─────────────────┐
                    │  SimulationEngine│
                    │  (orchestrator)  │
                    ├─────────────────┤
                    │  BodyGenerator   │  ← creates randomized bodies
                    │  SimLog          │  ← records snapshots
                    │  GroundPlane     │  ← collision response
                    └────────┬────────┘
                             │
               ┌─────────────┼─────────────┐
               ▼             ▼             ▼
        ┌──────────┐  ┌────────────┐  ┌──────────┐
        │Integrator│  │  RigidBody │  │SimConfig │
        ├──────────┤  ├────────────┤  ├──────────┤
        │ stepEuler│  │  x, q, P, L│  │ dt, steps│
        │ stepRK2  │  │  I_body    │  │ integrator│
        │ stepRK4  │  │  forces    │  │ seed     │
        │ stepSympl│  │  energy    │  └──────────┘
        └──────────┘  └────────────┘
                              │
              ┌───────────────┼───────────────┐
              ▼               ▼               ▼
        ┌──────────┐  ┌────────────┐  ┌──────────┐
        │  Vec3    │  │ Matrix3x3  │  │Quaternion│
        │ (vector) │  │  (tensor)  │  │ (rotation)│
        └──────────┘  └────────────┘  └──────────┘
```

### Dependency Graph

```
main.cpp
  └── Simulation.h
        ├── RigidBody.h
        │     ├── Vec3.h
        │     ├── Matrix3x3.h
        │     └── Quaternion.h
        │           ├── Vec3.h
        │           └── Matrix3x3.h
        └── Integrator.h
              └── RigidBody.h
```

---

## Running the Simulation

```bash
g++ -std=c++17 -O2 main.cpp -o build/main.out
./build/main.out
```

### Demo Configurations

The `main()` function runs two simulations back-to-back:

1. **RK4 with 3 bodies** — 20 steps at $\Delta t = 0.02$ s. Three randomized boxes fall under gravity with off-center impulses and ground collisions. Demonstrates multi-body simulation with the highest-accuracy integrator.

2. **Symplectic Euler with 1 body** — 20 steps at $\Delta t = 0.01$ s. A single body with the structure-preserving integrator. Shows bounded energy drift characteristics.

Both produce the same output format: initial conditions table, per-step state table with Unicode box-drawing characters, and an energy conservation summary at the end.

### Output Format

```
╔══════════════════════════════════════════════════════╗
║ UNCONSTRAINED RIGID BODY DYNAMICS — Baraff Model    ║
╠══════════════════════════════════════════════════════╣
║ Integrator: RK4 | dt = 0.0200 s | Steps: 20 | ...  ║
╚══════════════════════════════════════════════════════╝

╔══════════════════════════════════════════════════════╗
║ INITIAL CONDITIONS                                    ║
╚══════════════════════════════════════════════════════╝

  Body_0:
    Mass: 8.169 kg
    Position: [-2.400, 3.000, -0.325]
    Quaternion: [0.128, 0.355, -0.840, -0.391]
    Linear Mom: [21.495, 19.050, -21.742] kg*m/s
    ...

╔══════════════════════════════════════════════════════╗
║ Step │ Body │ Time  │ x(t)             │ v(t)  ... ║
╠══════════════════════════════════════════════════════╣
║ 0    │Body_0│ 0.000 │ [-2.40 3.00 -0.33]│ ...     ║
║ 1    │Body_0│ 0.020 │ [-2.35 3.04 -0.38]│ ...     ║
║ ...                                                
╚══════════════════════════════════════════════════════╝

╔══════════════════════════════════════════════════════╗
║ SIMULATION SUMMARY                                    ║
╠══════════════════════════════════════════════════════╣
║ Body_0                                                ║
║   Initial KE: 79.8773 J   Final KE: 69.0258 J       ║
║   ΔKE: -10.8516 J  (-13.59%)                         ║
║   Energy Drift Rate: -0.6793 %/step                  ║
╚══════════════════════════════════════════════════════╝
```

Each row in the state table shows:

| Column | Contents |
|--------|----------|
| Step | Integer step count |
| Body | Body identifier |
| Time | Simulation time (s) |
| x(t) | Center of mass position $[x, y, z]$ |
| v(t) | Linear velocity $[v_x, v_y, v_z]$ |
| w(t) | Angular velocity $[\omega_x, \omega_y, \omega_z]$ |
| KE (J) | Kinetic energy |

The summary reports initial vs. final kinetic energy, the change (absolute and percentage), and energy drift rate per step — a useful diagnostic for integrator performance.

---

## Key Equations at a Glance

| Quantity | Formula | Code |
|----------|---------|------|
| State vector | $\mathbf{Y} = [\mathbf{x}, \mathbf{q}, \mathbf{P}, \mathbf{L}]^T$ | `stateToArray` / `arrayToState` |
| Rotation matrix | $\mathbf{R} = f(\mathbf{q})$ | `Quaternion::toRotationMatrix()` |
| World inertia | $\mathbf{I}^{-1} = \mathbf{R}\,\mathbf{I}_{\text{body}}^{-1}\,\mathbf{R}^T$ | `RigidBody::computeAuxiliary()` |
| Linear velocity | $\mathbf{v} = \mathbf{P} / M$ | `RigidBody::computeAuxiliary()` |
| Angular velocity | $\boldsymbol{\omega} = \mathbf{I}^{-1}\,\mathbf{L}$ | `RigidBody::computeAuxiliary()` |
| Quaternion derivative | $\dot{\mathbf{q}} = \frac{1}{2}\boldsymbol{\omega}\,\mathbf{q}$ | `computeDerivatives()` |
| Kinetic energy | $E_k = \frac{1}{2}M\mathbf{v}^2 + \frac{1}{2}\boldsymbol{\omega}\cdot\mathbf{L}$ | `RigidBody::computeKineticEnergy()` |
| Torque from force | $\boldsymbol{\tau} = (\mathbf{p} - \mathbf{x}) \times \mathbf{F}$ | `RigidBody::applyForceAtPoint()` |

---

## Customization

The `SimulationConfig` struct exposes all control parameters:

```cpp
struct SimulationConfig {
    double dt = 0.02;              // Time step (seconds)
    int totalSteps = 30;           // Number of integration steps
    int numBodies = 3;             // Number of rigid bodies
    double gravity = 9.81;         // Gravitational acceleration (m/s^2)
    bool enableGround = true;      // Toggle ground collision
    bool enableRandomForces = true;// Apply transient off-center impulses
    IntegratorType integrator = IntegratorType::RK4;
    unsigned int seed = 42;        // RNG seed for reproducibility
};
```

To add a custom simulation, construct a `SimulationConfig`, fill in the fields, and pass it to `SimulationEngine`:

```cpp
SimulationConfig cfg;
cfg.dt = 0.005;
cfg.totalSteps = 100;
cfg.numBodies = 5;
cfg.integrator = IntegratorType::RK2_MIDPOINT;

SimulationEngine sim(cfg);
sim.run();
```

---

## What's Not Here (Yet)

This simulation covers unconstrained rigid body motion — bodies flying and tumbling through space with only ground contact. Several extensions from Baraff's full notes are left for future work:

- **Contact and constraints**: The ground collision is a simple impulse hack. A proper constraint-based solver (LCP or penalty-method) would handle stacking, resting contact, and multiple simultaneous contacts.
- **Collision detection for arbitrary shapes**: Currently spheres and boxes with ground-plane only. No box-box, box-sphere, or mesh-mesh collision.
- **Articulated bodies**: Joints (ball-in-socket, hinge, prismatic) connecting multiple bodies into kinematic chains.
- **Continuous collision detection**: CCD would prevent bodies from tunneling through the ground at high velocities.
- **Adaptive time stepping**: RK4 with fixed $\Delta t$ is wasteful during slow motion and risky during fast motion. An error-controlled RK45 or similar would be more robust.
- **Parallelism**: Each body's dynamics are independent between collisions — ideal for SIMD or multi-threaded execution.
- **Resting contact**: Bodies that come to rest on the ground currently micro-bounce. A proper resting contact algorithm would apply stabilizing forces to keep them still.

---

## References

1. **David Baraff**, *"Rigid Body Dynamics"*, SIGGRAPH 2001 Course Notes, Carnegie Mellon University. The theoretical foundation for everything in this simulation — quaternion kinematics, inertia tensor transformation, state-space formulation.

2. **Andrew Witkin and David Baraff**, *"Physically Based Modeling: Principles and Practice"*, SIGGRAPH 1997 Course Notes. The earlier exposition of the same material, with additional detail on numerical integration.

3. **David Eberly**, *"Game Physics"*, Morgan Kaufmann, 2nd Edition. A comprehensive reference covering rigid body dynamics, collision detection, and constraint solving.

4. **John H. Mathews and Kurtis K. Fink**, *"Numerical Methods Using MATLAB"*, Prentice Hall. For the Runge-Kutta theory and stability analysis.
