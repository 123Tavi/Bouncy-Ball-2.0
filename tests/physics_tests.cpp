#include "Physics.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

using bouncy::Ball;
using bouncy::PhysicsWorld;
using bouncy::WorldConfig;

int failures = 0;

void check(bool cond, const char* msg) {
    if (!cond) {
        std::cerr << "[FAIL] " << msg << "\n";
        ++failures;
    } else {
        std::cout << "[PASS] " << msg << "\n";
    }
}

void testFreeFallKinematics() {
    WorldConfig cfg{};
    cfg.airDragEnabled = false;
    cfg.magnusEnabled = false;
    cfg.gravity = {0.0f, -9.81f};
    cfg.subSteps = 8;

    PhysicsWorld world(cfg);
    const float y0 = 6.0f;
    world.addBall({4.0f, y0}, {0.0f, 0.0f}, 0.20f, 0);

    // Integrate 0.8 seconds (48 frames at 60 Hz) before hitting floor
    constexpr int kFrames = 48;
    constexpr float kDt = 1.0f / 60.0f;
    for (int i = 0; i < kFrames; ++i) {
        world.step(kDt);
    }

    const float t = kFrames * kDt;
    const float expectedY = y0 - 0.5f * 9.81f * t * t;
    const float expectedVy = -9.81f * t;
    const Ball& b = world.balls().front();

    check(std::abs(b.pos.y - expectedY) < 1e-3f,
          "Free-fall position y(t) = y0 - 0.5*g*t^2 accurate within 1 mm");
    check(std::abs(b.vel.y - expectedVy) < 1e-3f,
          "Free-fall velocity v_y(t) = -g*t accurate within 1 mm/s");
}

void testElasticBounceEnergyConservation() {
    WorldConfig cfg{};
    cfg.airDragEnabled = false;
    cfg.magnusEnabled = false;
    cfg.useGlobalRestitution = true;
    cfg.globalRestitution = 1.0f;
    cfg.gravity = {0.0f, -9.81f};
    cfg.subSteps = 8;

    PhysicsWorld world(cfg);
    Ball& b = world.addBall({4.0f, 4.20f}, {0.0f, 0.0f}, 0.20f, 0);
    b.friction = 0.0f;

    const float initialEnergy = world.computeSystemEnergy().total;

    // Simulate 16 seconds (t_fall = 0.903s, bounce period = 1.806s -> ~9 bounces)
    for (int i = 0; i < 960; ++i) {
        world.step(1.0f / 60.0f);
    }

    const float finalEnergy = world.computeSystemEnergy().total;
    const float relErr = std::abs(finalEnergy - initialEnergy) / initialEnergy;

    check(world.balls().front().bounceCount >= 8,
          "Elastic ball completed at least 8 floor bounces");
    check(relErr < 0.002f,
          "Total mechanical energy E = KE + PE conserved within 0.2% across 16s of bounces");
}

void testRestitutionApexLaw() {
    // Without drag, h_{n+1} = e^2 * h_n. For e = 0.8, h_1 = 0.64 * h_0!
    WorldConfig cfg{};
    cfg.airDragEnabled = false;
    cfg.magnusEnabled = false;
    cfg.useGlobalRestitution = true;
    cfg.globalRestitution = 0.80f;
    cfg.gravity = {0.0f, -9.81f};
    cfg.subSteps = 10;

    PhysicsWorld world(cfg);
    const float r = 0.20f;
    const float h0 = 4.00f; // 4.0 m above floor contact
    Ball& b = world.addBall({4.0f, h0 + r}, {0.0f, 0.0f}, r, 0);
    b.friction = 0.0f;

    // Run until 2nd floor bounce completes so lastBounceApex records apex of bounce #1
    while (world.balls().front().bounceCount < 2) {
        world.step(1.0f / 120.0f);
    }

    const float expectedFirstReboundApex = 0.80f * 0.80f * h0; // 2.56 m
    const float measuredApex = world.balls().front().lastBounceApex;
    check(std::abs(measuredApex - expectedFirstReboundApex) < 0.015f,
          "Inelastic bounce apex matches h_1 = e^2 * h_0 (2.56m for h_0=4m, e=0.8)");
}

void testTwoBallElasticCollision() {
    WorldConfig cfg{};
    cfg.airDragEnabled = false;
    cfg.magnusEnabled = false;
    cfg.useGlobalRestitution = true;
    cfg.globalRestitution = 1.0f;
    cfg.gravity = {0.0f, 0.0f}; // Zero-G isolated collision test
    cfg.subSteps = 8;

    PhysicsWorld world(cfg);
    world.addBall({3.0f, 4.0f}, {4.0f, 0.0f}, 0.25f, 0);
    world.addBall({5.0f, 4.0f}, {-2.0f, 0.0f}, 0.25f, 0);
    world.balls()[0].friction = 0.0f;
    world.balls()[1].friction = 0.0f;

    const float p0 = world.balls()[0].mass * world.balls()[0].vel.x
                   + world.balls()[1].mass * world.balls()[1].vel.x;
    const float e0 = world.computeSystemEnergy().total;

    for (int i = 0; i < 60; ++i) {
        world.step(1.0f / 60.0f);
    }

    const auto& balls = world.balls();
    const float p1 = balls[0].mass * balls[0].vel.x + balls[1].mass * balls[1].vel.x;
    const float e1 = world.computeSystemEnergy().total;

    // Equal-mass head-on 1D elastic collision swaps velocities: vA' = -2, vB' = +4
    check(std::abs(p1 - p0) < 1e-4f, "Two-ball collision conserves linear momentum");
    check(std::abs(e1 - e0) / e0 < 1e-4f, "Two-ball elastic collision conserves kinetic energy");
    check(std::abs(balls[0].vel.x - (-2.0f)) < 1e-2f && std::abs(balls[1].vel.x - 4.0f) < 1e-2f,
          "Equal-mass head-on elastic collision swaps velocities cleanly");
}

} // namespace

int main() {
    std::cout << "Running Bouncy-Ball 3.0 Physics Test Suite...\n";
    testFreeFallKinematics();
    testElasticBounceEnergyConservation();
    testRestitutionApexLaw();
    testTwoBallElasticCollision();

    if (failures > 0) {
        std::cerr << failures << " test(s) failed.\n";
        return EXIT_FAILURE;
    }
    std::cout << "All physics tests passed!\n";
    return EXIT_SUCCESS;
}
