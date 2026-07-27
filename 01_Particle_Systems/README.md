# Particle System Dynamics

> *"Everything should be made as simple as possible, but not simpler."* -- Albert Einstein

This is a Newtonian particle dynamics simulation engine written in C++. It solves the equations of motion for point masses connected by damped springs, falling under gravity, and resisting through viscous drag -- all integrated forward in time using second-order Runge-Kutta.

Think of it as a tiny physics sandbox: throw some masses into the world, wire them together with springs, let gravity do its thing, and watch the ODE solver figure out where everything ends up.

---

## The Physics

At its core, this simulation is an ODE solver dressed up in Newton's clothing. Every particle obeys Newton's second law:

$$\mathbf{F} = m\mathbf{a} \quad \Longrightarrow \quad m\ddot{\mathbf{x}} = \sum_i \mathbf{F}_i(\mathbf{x}, \dot{\mathbf{x}})$$

That single vector equation is the entire universe of this simulation. Every force -- gravity, drag, spring tension -- just feeds into that sum. The solver's job is to figure out where the particle goes next.

### Equation of Motion

We cast the second-order ODE into a system of first-order ODEs by defining the state vector:

$$\mathbf{s} = \begin{pmatrix} \mathbf{x} \\ \mathbf{v} \end{pmatrix}$$

For $N$ particles, the full state lives in $\mathbb{R}^{6N}$, with each particle contributing 3 position components and 3 velocity components. The derivative is then:

$$\dot{\mathbf{s}} = \begin{pmatrix} \dot{\mathbf{x}} \\ \dot{\mathbf{v}} \end{pmatrix} = \begin{pmatrix} \mathbf{v} \\ \mathbf{F}(\mathbf{x}, \mathbf{v}) / m \end{pmatrix}$$

This is the standard first-order reformulation -- it's what every numerical integrator in the code operates on. The `ParticleSystem::derivative()` method is the right-hand side of this ODE, and it's the beating heart of the whole simulation.

### Forces

Three force types are implemented. Each one plugs into the universal force accumulator via a polymorphic `apply()` method.

---

#### 1. Uniform Gravitational Field

The simplest force in physics, and still the one that keeps your feet on the ground:

$$\mathbf{F}_{\text{grav}} = m\mathbf{g}$$

where $\mathbf{g} = (0, -9.81, 0) \, \text{m/s}^2$. This is a flat-Earth approximation -- the gravitational field is constant everywhere, pointing downward. It's not the inverse-square law from Newton's law of gravitation; it's the linearized version you use when you're standing on a planet and not orbiting one.

In the code, `GravityForce` loops over every particle and adds $m_i \mathbf{g}$ to its force accumulator. Simple, effective, and exactly what you'd expect.

---

#### 2. Linear Viscous Drag

When a particle moves through a medium (air, water, honey), it experiences resistance. At low Reynolds numbers, this drag is proportional to velocity:

$$\mathbf{F}_{\text{drag}} = -k_d \, \mathbf{v}$$

where $k_d$ is the drag coefficient. This is a **dissipative force** -- it always opposes motion and extracts kinetic energy from the system. In the long run, any freely falling particle with drag will reach terminal velocity:

$$v_{\text{terminal}} = \frac{mg}{k_d}$$

The simulation uses $k_d = 0.1 \, \text{N}{\cdot}\text{s/m}$. For a 1 kg particle, that gives a terminal velocity of $98.1 \, \text{m/s}$ -- fast, but not infinite. Drag keeps things from accelerating forever, which is physically comforting.

---

#### 3. Damped Spring (Hooke + Dashpot)

This is where it gets interesting. A spring connecting two particles $p_1$ and $p_2$ exerts a force that depends on both the displacement and the rate of deformation. The full force law is:

$$F_{\text{spring}} = -\left[k_s (l - l_0) + k_d \, \dot{l}\right] \hat{\mathbf{l}}$$

where:

| Symbol | Meaning |
|--------|---------|
| $l = \|\mathbf{x}_1 - \mathbf{x}_2\|$ | Current distance between particles |
| $l_0$ | Rest length (equilibrium) |
| $\dot{l}$ | Rate of change of length (relative velocity along the spring axis) |
| $\hat{\mathbf{l}}$ | Unit vector from $p_2$ to $p_1$ |
| $k_s$ | Spring constant (stiffness) |
| $k_d$ | Damping constant (dashpot coefficient) |

This is a **Kelvin-Voigt model**: a linear spring in parallel with a viscous damper. The $k_s$ term is Hooke's law -- the restoring force that makes springs bounce. The $k_d$ term is the dashpot -- it removes energy from the oscillation, making it decay over time.

The force on $p_1$ is $+F_{\text{spring}} \hat{\mathbf{l}}$ and on $p_2$ is $-F_{\text{spring}} \hat{\mathbf{l}}$ -- Newton's third law in action. The spring doesn't care which end is which; it just pulls or pushes equally on both sides.

**Parameters used in the demo:**

| Parameter | Value | Physical Meaning |
|-----------|-------|------------------|
| $k_s$ | 50 N/m | Stiffness -- how hard the spring resists stretching |
| $k_d$ | 0.5 N$\cdot$s/m | Damping -- how quickly oscillations die out |
| $l_0$ | 3 m | Natural length -- the spring's favorite distance |

With these values, the natural frequency of the spring is $\omega_n = \sqrt{k_s / \mu}$ where $\mu$ is the reduced mass. For two 1 kg particles, $\mu = 0.5$ kg, giving $\omega_n = 10$ rad/s and a period of about $0.63$ s. The damping ratio is $\zeta = k_d / (2\sqrt{k_s \mu}) = 0.05$ -- an underdamped spring that oscillates roughly 20 times before the amplitude drops to $1/e$ of its initial value.

---

## Numerical Integration

The simulation provides two integrators. The one actually used is the **midpoint method** (RK2), which is a good balance between simplicity and accuracy.

### Forward Euler (Explicit Euler)

The simplest possible integrator:

$$\mathbf{s}_{n+1} = \mathbf{s}_n + \Delta t \, \dot{\mathbf{s}}_n$$

This is first-order accurate: the global error is $O(\Delta t)$. It's easy to implement but **energy-unstable** for oscillatory systems -- it tends to inject energy into springs, causing them to blow up over time. It's included for reference but not used in the demo.

### Midpoint Method (RK2 -- Modified Euler)

A two-stage method that's significantly better:

$$\begin{aligned}
\mathbf{k}_1 &= \dot{\mathbf{s}}(\mathbf{s}_n) \\
\mathbf{k}_2 &= \dot{\mathbf{s}}(\mathbf{s}_n + \tfrac{1}{2}\Delta t \, \mathbf{k}_1) \\
\mathbf{s}_{n+1} &= \mathbf{s}_n + \Delta t \, \mathbf{k}_2
\end{aligned}$$

This is second-order accurate: $O(\Delta t^2)$ global error. The trick is that $\mathbf{k}_2$ evaluates the derivative at the **midpoint** of the step, not at the beginning. This symmetry makes it much better at conserving energy over long simulations -- it won't pump energy into springs the way Euler does.

The simulation uses $\Delta t = 0.01$ s with 200 steps, totaling 2 seconds of simulated time. At this step size, the midpoint method gives smooth, stable trajectories for the spring-mass-damper system.

### Why Not RK4?

The classical fourth-order Runge-Kutta method would give $O(\Delta t^4)$ accuracy, but for this simple two-particle system, RK2 is more than sufficient. RK4 would be overkill -- like driving a Ferrari to the mailbox. But the architecture supports it: just write a new static method in `Solver` that works on the same `getState`/`setState`/`derivative` interface.

---

## The Code

```
main.cpp           -- Simulation setup, time loop, output
Particle.h         -- Point mass: position, velocity, force accumulator
ParticleSystem.h   -- Container: particles, forces, state vector, ODE RHS
Vec3.h             -- 3D vector arithmetic
Force.h            -- Abstract force + Gravity, Drag, Spring implementations
Solver.h           -- Euler and Midpoint integrators
```

### Architecture

The design is deliberately decoupled. The `ParticleSystem` manages state, the `Force` hierarchy defines physics, and the `Solver` performs integration -- they don't know about each other's internals.

```
                    ┌──────────────┐
                    │  Solver      │
                    │  (RK2/Euler) │
                    └──────┬───────┘
                           │ calls getState / setState / derivative
                           ▼
                ┌─────────────────────┐
                │  ParticleSystem     │
                │  (state container)  │
                ├─────────────────────┤
                │  Particle[]         │  ← point masses
                │  Force[]            │  ← polymorphic forces
                │    ├─ GravityForce  │
                │    ├─ DragForce     │
                │    └─ SpringForce   │
                └─────────────────────┘
```

The key insight: the solver doesn't care whether it's integrating 2 particles or 2000. It just sees a state vector in $\mathbb{R}^{6N}$ and a function that computes its derivative. This is the **strategy pattern** applied to physics.

### State Vector Layout

The flattened state vector is organized as:

$$\mathbf{s} = [x_1, y_1, z_1, v_{x1}, v_{y1}, v_{z1}, \, x_2, y_2, z_2, v_{x2}, v_{y2}, v_{z2}, \, \ldots]$$

Each particle occupies 6 consecutive slots. `getState()` reads positions and velocities from the particle objects into this vector; `setState()` writes them back. The derivative function then fills in $\dot{\mathbf{s}} = [\mathbf{v}_1, \mathbf{F}_1/m_1, \mathbf{v}_2, \mathbf{F}_2/m_2, \ldots]$.

---

## Running the Simulation

```bash
g++ -std=c++11 main.cpp -o particle_simulation
./particle_simulation
```

The output is a time-series table showing particle positions at 0.2 s intervals:

```
Time(s) | P1 Pos (X, Y, Z)        | P2 Pos (X, Y, Z)
--------------------------------------------------------
0.000s  | (0.000, 10.000, 0.000)  | (0.000, 5.000, 0.000)
0.200s  | (0.000, 8.477, 0.000)   | (0.000, 6.133, 0.000)
...
2.000s  | (0.000, -9.224, 0.000)  | (0.000, -12.526, 0.000)
```

Two particles, initially at rest, fall under gravity while connected by a damped spring. You can watch them oscillate, stretch, compress, and eventually settle toward terminal velocity as drag and spring damping dissipate energy.

---

## What's Not Here (Yet)

This is Chapter 1 -- the foundation. Several obvious extensions are left for future work:

- **Collision detection**: Particles can pass through each other. There's no contact force or collision response.
- **Boundary conditions**: Particles can fly off to infinity. No walls, no floor, no constraints.
- **Adaptive time stepping**: The step size is fixed at $\Delta t = 0.01$ s. Stiffer springs would require smaller steps for stability.
- **Symplectic integrators**: A velocity-Verlet or leapfrog integrator would conserve energy better for Hamiltonian systems.
- **Inverse-square gravity**: Currently uniform; could be extended to $F = GMm/r^2$ for orbital mechanics.
- **Rotational dynamics**: Particles are point masses with no orientation or moment of inertia.

---

