#ifndef SIMULATION_H
#define SIMULATION_H

#include <iostream>
#include <vector>
#include <iomanip>
#include <random>
#include <string>
#include <sstream>
#include <algorithm>
#include <functional>
#include "RigidBody.h"
#include "Integrator.h"

// ============================================================================
// GROUND COLLISION
// ============================================================================
struct GroundPlane {
    double y = 0.0;
    double restitution = 0.5;
    double friction = 0.3;
};

inline void handleGroundCollision(RigidBody& rb, const GroundPlane& ground) {
    // The floor is immovable, so only the body's inverse mass enters the impulse.
    if (rb.x.y < ground.y) {
        rb.x.y = ground.y + 1e-6;

        double vn = rb.v.y;
        if (vn < 0.0) {
            double jn = -(1.0 + ground.restitution) * vn / (1.0 / rb.mass);
            rb.P.y += jn;
            rb.v.y = rb.P.y / rb.mass;
        }

        rb.v.x *= (1.0 - ground.friction * 0.05);
        rb.v.z *= (1.0 - ground.friction * 0.05);
        rb.P.x = rb.mass * rb.v.x;
        rb.P.z = rb.mass * rb.v.z;

        rb.omega *= (1.0 - ground.friction * 0.02);
        rb.L = rb.Iinv * rb.omega;
    }
}

// ============================================================================
// SIMULATION CONFIGURATION
// ============================================================================
struct SimulationConfig {
    double dt = 0.02;
    int totalSteps = 30;
    int numBodies = 3;
    double gravity = 9.81;
    bool enableGround = true;
    bool enableRandomForces = true;
    IntegratorType integrator = IntegratorType::RK4;
    unsigned int seed = 42;
};

// ============================================================================
// BODY GENERATOR
// ============================================================================
struct BodyGenerator {
    std::mt19937 rng;

    BodyGenerator(unsigned int seed) : rng(seed) {}

    RigidBody createRandomBox(int id) {
        RigidBody body;
        body.id = id;
        body.name = "Body_" + std::to_string(id);

        std::uniform_real_distribution<double> dist_mass(1.0, 10.0);
        std::uniform_real_distribution<double> dist_dim(0.3, 2.0);
        std::uniform_real_distribution<double> dist_pos(-3.0, 3.0);
        std::uniform_real_distribution<double> dist_vel(-3.0, 3.0);
        std::uniform_real_distribution<double> dist_omega(-2.0, 2.0);
        std::uniform_real_distribution<double> dist_angle(0.0, 2.0 * M_PI);

        body.mass = dist_mass(rng);
        double w = dist_dim(rng), h = dist_dim(rng), d = dist_dim(rng);
        body.setBoxInertia(w, h, d);

        body.x = Vec3(dist_pos(rng), 3.0 + id * 2.0, dist_pos(rng));
        body.q = Quaternion::fromAxisAngle(
            Vec3(dist_omega(rng), dist_omega(rng), dist_omega(rng)).normalized(),
            dist_angle(rng)
        );
        body.P = Vec3(dist_vel(rng), dist_vel(rng) + 1.0, dist_vel(rng)) * body.mass;

        Vec3 omega_init(dist_omega(rng), dist_omega(rng), dist_omega(rng));
        body.computeAuxiliary();
        body.L = body.Iinv * omega_init;

        body.computeAuxiliary();
        return body;
    }
};

// ============================================================================
// SIMULATION STATE LOGGER
// ============================================================================
struct SimSnapshot {
    int step;
    double time;
    int bodyId;
    std::string bodyName;
    Vec3 x;
    Vec3 v;
    Vec3 omega;
    Vec3 P;
    Vec3 L;
    double KE;
};

struct SimLog {
    std::vector<SimSnapshot> snapshots;

    void record(int step, double time, const RigidBody& rb) {
        SimSnapshot snap;
        snap.step = step;
        snap.time = time;
        snap.bodyId = rb.id;
        snap.bodyName = rb.name;
        snap.x = rb.x;
        snap.v = rb.v;
        snap.omega = rb.omega;
        snap.P = rb.P;
        snap.L = rb.L;
        snap.KE = rb.computeKineticEnergy();
        snapshots.push_back(snap);
    }

    void clear() { snapshots.clear(); }
};

// ============================================================================
// FORMATTED OUTPUT TOOLS
// ============================================================================
namespace Table {
    const std::string HDR  = "═";
    const std::string VBR  = "║";
    const std::string TL   = "╔";
    const std::string TR   = "╗";
    const std::string BL   = "╚";
    const std::string BR   = "╝";
    const std::string TEE  = "╤";
    const std::string BEE  = "╧";
    const std::string LTE  = "╠";
    const std::string RTE  = "╣";
    const std::string MID  = "┼";
    const std::string HDR2 = "─";

    inline std::string repeat(const std::string& s, int n) {
        std::string r;
        for (int i = 0; i < n; ++i) r += s;
        return r;
    }

    inline int tableWidth(const std::vector<int>& widths) {
        int sum = 0;
        for (int w : widths) sum += w;
        return sum + 3 * (int)widths.size() + 1;
    }

    inline void printVLine() { std::cout << VBR; }

    inline void printTopBorder(const std::vector<int>& widths) {
        std::cout << TL;
        for (size_t i = 0; i < widths.size(); ++i) {
            std::cout << repeat(HDR2, widths[i] + 2);
            if (i < widths.size() - 1) std::cout << TEE; else std::cout << TR;
        }
        std::cout << "\n";
    }

    inline void printMidBorder(const std::vector<int>& widths) {
        std::cout << LTE;
        for (size_t i = 0; i < widths.size(); ++i) {
            std::cout << repeat(HDR2, widths[i] + 2);
            if (i < widths.size() - 1) std::cout << MID; else std::cout << RTE;
        }
        std::cout << "\n";
    }

    inline void printBotBorder(const std::vector<int>& widths) {
        std::cout << BL;
        for (size_t i = 0; i < widths.size(); ++i) {
            std::cout << repeat(HDR2, widths[i] + 2);
            if (i < widths.size() - 1) std::cout << BEE; else std::cout << BR;
        }
        std::cout << "\n";
    }

    inline void printHeaderRow(const std::vector<std::string>& headers, const std::vector<int>& widths) {
        printVLine();
        for (size_t i = 0; i < headers.size(); ++i) {
            int pad = (widths[i] - (int)headers[i].length()) / 2;
            int pad_r = widths[i] - pad - (int)headers[i].length();
            std::cout << " " << std::string(pad, ' ') << headers[i]
                      << std::string(pad_r, ' ') << " ";
            printVLine();
        }
        std::cout << "\n";
    }

    inline void printDataRow(const std::vector<std::string>& cells, const std::vector<int>& widths) {
        printVLine();
        for (size_t i = 0; i < cells.size(); ++i) {
            int w = (i < widths.size()) ? widths[i] : 8;
            int pad = w - (int)cells[i].length();
            if (pad < 0) pad = 0;
            std::cout << " " << cells[i] << std::string(pad, ' ') << " ";
            printVLine();
        }
        std::cout << "\n";
    }

    inline void printBoxTop(int innerWidth) {
        std::cout << TL << repeat(HDR, innerWidth) << TR << "\n";
    }

    inline void printBoxMid(int innerWidth) {
        std::cout << LTE << repeat(HDR2, innerWidth) << RTE << "\n";
    }

    inline void printBoxBot(int innerWidth) {
        std::cout << BL << repeat(HDR2, innerWidth) << BR << "\n";
    }

    inline void printBoxLine(const std::string& content, int innerWidth) {
        int pad = innerWidth - (int)content.length();
        if (pad < 0) pad = 0;
        std::cout << VBR << " " << content << std::string(pad, ' ') << " " << VBR << "\n";
    }

    inline std::string fmtVec(const Vec3& v, int w = 7, int prec = 2) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(prec)
           << "[" << std::setw(w) << v.x << " " << std::setw(w) << v.y << " " << std::setw(w) << v.z << "]";
        return ss.str();
    }

    inline std::string fmtNum(double v, int w = 8, int prec = 3) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(prec) << std::setw(w) << v;
        return ss.str();
    }

    struct ColumnDef {
        std::string header;
        int width;
        std::function<std::string(const RigidBody&)> extractor;
    };
}

// ============================================================================
// SIMULATION ENGINE
// ============================================================================
struct SimulationEngine {
    SimulationConfig cfg;
    std::vector<RigidBody> bodies;
    GroundPlane ground;
    SimLog log;
    BodyGenerator gen;

    SimulationEngine(const SimulationConfig& config)
        : cfg(config), ground{0.0, 0.5, 0.3}, gen(config.seed) {}

    void initialize() {
        bodies.clear();
        log.clear();
        for (int i = 0; i < cfg.numBodies; ++i) {
            bodies.push_back(gen.createRandomBox(i));
        }
    }

    void applyForces(RigidBody& body, int step) {
        body.resetAccumulators();
        body.applyForceAtCOM(Vec3(0, -cfg.gravity * body.mass, 0));
        if (cfg.enableRandomForces && step < 5) {
            std::uniform_real_distribution<double> dist_force(-15.0, 15.0);
            std::uniform_real_distribution<double> dist_off(-0.8, 0.8);
            Vec3 F(dist_force(gen.rng), dist_force(gen.rng), dist_force(gen.rng));
            Vec3 offset(dist_off(gen.rng), dist_off(gen.rng), dist_off(gen.rng));
            Vec3 world_p = body.x + body.R * offset;
            body.applyForceAtPoint(F, world_p);
        }
    }

    std::vector<int> colWidths() const {
        return {6, 8, 10, 28, 28, 28, 12};
    }

    int tableInnerWidth() const {
        auto w = colWidths();
        return Table::tableWidth(w) - 2;
    }

    void run() {
        initialize();

        auto widths = colWidths();
        int innerW = tableInnerWidth();

        // Title block
        Table::printBoxTop(innerW);
        Table::printBoxLine("UNCONSTRAINED RIGID BODY DYNAMICS -- Enhanced Simulator (Baraff Model)", innerW);
        Table::printBoxMid(innerW);
        {
            std::ostringstream ss;
            ss << "Integrator: " + integratorName(cfg.integrator)
               + " | dt = " + Table::fmtNum(cfg.dt, 4, 4) + " s"
               + " | Steps: " + std::to_string(cfg.totalSteps)
               + " | Bodies: " + std::to_string(cfg.numBodies)
               + " | Ground: " + (cfg.enableGround ? "ON" : "OFF")
               + " | Seed: " + std::to_string(cfg.seed);
            Table::printBoxLine(ss.str(), innerW);
        }
        Table::printBoxBot(innerW);
        std::cout << "\n";

        // Initial conditions
        Table::printBoxTop(innerW);
        Table::printBoxLine("INITIAL CONDITIONS", innerW);
        Table::printBoxBot(innerW);
        std::cout << "\n";

        for (const auto& body : bodies) {
            std::cout << "  " << body.name << ":\n";
            std::cout << "    Mass: " << std::fixed << std::setprecision(3) << body.mass << " kg\n";
            std::cout << "    Position: " << body.x << "\n";
            std::cout << "    Orientation (q): " << body.q << "\n";
            std::cout << "    Linear Mom: " << body.P << " kg*m/s\n";
            std::cout << "    Angular Mom: " << body.L << " kg*m^2/s\n";
            std::cout << "    Kinetic Energy: " << std::fixed << std::setprecision(4) << body.computeKineticEnergy() << " J\n\n";
        }

        // Column definitions
        std::vector<Table::ColumnDef> cols = {
            {"Step", 6, [](const RigidBody&) { return ""; }},
            {"Body", 8, [](const RigidBody& b) { return b.name; }},
            {"Time (s)", 10, [](const RigidBody&) { return ""; }},
            {"x(t)", 28, [](const RigidBody& b) { return Table::fmtVec(b.x, 6, 2); }},
            {"v(t)", 28, [](const RigidBody& b) { return Table::fmtVec(b.v, 6, 2); }},
            {"w(t)", 28, [](const RigidBody& b) { return Table::fmtVec(b.omega, 6, 2); }},
            {"KE (J)", 12, [](const RigidBody& b) { return Table::fmtNum(b.computeKineticEnergy(), 8, 4); }},
        };

        std::vector<std::string> headers;
        for (auto& c : cols) headers.push_back(c.header);

        bool header_printed = false;

        for (int step = 0; step <= cfg.totalSteps; ++step) {
            double time = step * cfg.dt;

            if (!header_printed) {
                Table::printTopBorder(widths);
                Table::printHeaderRow(headers, widths);
                header_printed = true;
            }

            Table::printMidBorder(widths);

            for (auto& body : bodies) {
                log.record(step, time, body);

                std::string time_str = Table::fmtNum(time, 6, 3);
                std::vector<std::string> cells;
                for (auto& c : cols) {
                    if (c.header == "Step") cells.push_back(std::to_string(step));
                    else if (c.header == "Time (s)") cells.push_back(time_str);
                    else cells.push_back(c.extractor(body));
                }
                Table::printDataRow(cells, widths);
            }

            for (auto& body : bodies) {
                applyForces(body, step);
            }

            if (step == cfg.totalSteps) break;

            for (auto& body : bodies) {
                stepIntegrator(body, cfg.dt, cfg.integrator);
                if (cfg.enableGround) {
                    handleGroundCollision(body, ground);
                }
            }
        }

        Table::printBotBorder(widths);
        std::cout << "\n";

        printSummary(innerW);
    }

    void printSummary(int innerW) {
        Table::printBoxTop(innerW);
        Table::printBoxLine("SIMULATION SUMMARY", innerW);
        Table::printBoxMid(innerW);

        for (auto& body : bodies) {
            auto first = std::find_if(log.snapshots.begin(), log.snapshots.end(),
                [&](const SimSnapshot& s) { return s.bodyId == body.id && s.step == 0; });
            auto last = std::find_if(log.snapshots.rbegin(), log.snapshots.rend(),
                [&](const SimSnapshot& s) { return s.bodyId == body.id; });

            if (first == log.snapshots.end() || last == log.snapshots.rend()) continue;

            double KE_i = first->KE, KE_f = last->KE;
            double KE_d = KE_f - KE_i;
            double KE_p = (KE_i != 0.0) ? (KE_d / KE_i) * 100.0 : 0.0;

            double v_i = first->v.length(), v_f = last->v.length();
            double w_i = first->omega.length(), w_f = last->omega.length();
            double P_i = first->P.length(), P_f = last->P.length();
            double L_i = first->L.length(), L_f = last->L.length();

            double drift = (cfg.totalSteps > 0 && KE_i != 0)
                ? ((KE_f - KE_i) / KE_i) * 100.0 / cfg.totalSteps : 0.0;

            // Build each line as a string, pad to innerW
            Table::printBoxLine(body.name, innerW);
            {
                std::ostringstream ss;
                ss << std::fixed;
                ss << "   Initial KE: " << std::setw(10) << std::setprecision(4) << KE_i
                   << " J   Final KE: " << std::setw(10) << KE_f
                   << " J   " << (KE_d >= 0 ? "+" : "") << std::setw(10) << KE_d
                   << " J  (" << std::setw(6) << std::setprecision(2) << KE_p << "%)";
                Table::printBoxLine(ss.str(), innerW);
            }
            {
                std::ostringstream ss;
                ss << std::fixed;
                ss << "   |v|: " << std::setw(8) << std::setprecision(4) << v_i
                   << " -> " << std::setw(8) << v_f
                   << "   |w|: " << std::setw(8) << w_i
                   << " -> " << std::setw(8) << w_f
                   << "   |P|: " << std::setw(8) << P_i
                   << " -> " << std::setw(8) << P_f;
                Table::printBoxLine(ss.str(), innerW);
            }
            {
                std::ostringstream ss;
                ss << std::fixed;
                ss << "   |L|: " << std::setw(8) << std::setprecision(4) << L_i
                   << " -> " << std::setw(8) << L_f;
                Table::printBoxLine(ss.str(), innerW);
            }
            Table::printBoxLine("", innerW);
            {
                std::ostringstream ss;
                ss << std::fixed;
                ss << "   Energy Drift Rate: "
                   << std::setw(10) << std::setprecision(4) << drift
                   << " %/step   (" << integratorName(cfg.integrator) << ")";
                Table::printBoxLine(ss.str(), innerW);
            }
            Table::printBoxMid(innerW);
        }

        Table::printBoxBot(innerW);
        std::cout << "\n";
    }
};

#endif
