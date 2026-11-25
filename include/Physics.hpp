#pragma once

#include "Vec2.hpp"

#include <array>
#include <cstdint>
#include <deque>
#include <string_view>
#include <vector>

namespace bouncy {

struct Rgba {
    std::uint8_t r{255};
    std::uint8_t g{255};
    std::uint8_t b{255};
    std::uint8_t a{255};
};

struct Material {
    std::string_view name;
    float density;      // kg / m^2 (2D areal density)
    float restitution;  // coefficient of restitution e in [0, 1]
    float friction;     // Coulomb friction coefficient mu
    float dragCoeff;    // aerodynamic drag coefficient Cd
    Rgba color;
};

inline constexpr std::array<Material, 6> kMaterials{{
    {"Lab SkyBlue", 16.0f, 0.80f, 0.25f, 0.47f, {102, 191, 255, 255}},
    {"Superball",   12.0f, 0.93f, 0.55f, 0.45f, {255,  92, 141, 255}},
    {"Steel",       78.0f, 0.62f, 0.18f, 0.42f, {195, 208, 224, 255}},
    {"Tennis",       7.5f, 0.76f, 0.48f, 0.58f, {198, 245,  78, 255}},
    {"Basketball",  10.5f, 0.84f, 0.42f, 0.50f, {255, 152,  56, 255}},
    {"Heavy Lead", 113.0f, 0.25f, 0.35f, 0.44f, {136, 126, 168, 255}},
}};

struct TrailPoint {
    Vec2 pos;
    float speed{0.0f};
};

struct Ball {
    std::uint32_t id{0};
    Vec2 pos{};                // meters, world space (+x right, +y up, y=0 at floor)
    Vec2 vel{};                // m/s
    float radius{0.20f};       // meters
    float mass{2.0f};          // kg
    float inertia{0.04f};      // kg * m^2 (solid disk: 0.5 * m * r^2)
    float angle{0.0f};         // radians
    float angularVel{0.0f};    // rad/s

    float restitution{0.80f};
    float friction{0.25f};
    float dragCoeff{0.47f};
    std::size_t materialIdx{0};
    Rgba color{102, 191, 255, 255};

    // Bounce & apex telemetry (fulfills & upgrades original 2.0 height tracking)
    float initialHeight{0.0f};        // Center height above floor (m) when spawned/dropped
    float currentCyclePeak{0.0f};     // Highest center height (m) since last floor contact
    float lastBounceApex{0.0f};       // Measured apex height (m above radius) of previous bounce
    float prevBounceApex{0.0f};       // Measured apex height before lastBounceApex
    float lastApexDiff{0.0f};         // prevBounceApex - lastBounceApex (m)
    float theoreticalNextApex{0.0f};  // v_y^2 / (2 * |g_y|) right after floor bounce
    std::uint32_t bounceCount{0};

    // Visual squash-and-stretch state
    Vec2 squashNormal{0.0f, 1.0f};
    float squash{0.0f};        // [0, 0.38] compression along squashNormal
    float squashVel{0.0f};

    std::deque<TrailPoint> trail;
    float trailSampleTimer{0.0f};

    // Energies relative to floor contact (h = pos.y - radius >= 0)
    [[nodiscard]] float heightAboveFloor() const noexcept {
        const float h = pos.y - radius;
        return h > 0.0f ? h : 0.0f;
    }

    [[nodiscard]] float translationalKE() const noexcept {
        return 0.5f * mass * vel.lengthSq();
    }

    [[nodiscard]] float rotationalKE() const noexcept {
        return 0.5f * inertia * angularVel * angularVel;
    }

    [[nodiscard]] float potentialEnergy(float gravityMag) const noexcept {
        return mass * gravityMag * heightAboveFloor();
    }
};

struct Ramp {
    Vec2 a{};
    Vec2 b{};
    float thickness{0.05f}; // meters (capsule radius)
    float restitution{0.85f};
    float friction{0.35f};
};

struct ImpactEvent {
    Vec2 pos{};
    Vec2 normal{0.0f, 1.0f};
    float normalSpeed{0.0f};   // m/s
    float impulse{0.0f};       // N * s
    float radius{0.20f};       // m
    Rgba color{};
};

struct EnergySnapshot {
    float translationalKE{0.0f};
    float rotationalKE{0.0f};
    float potential{0.0f};
    float total{0.0f};
};

struct WorldConfig {
    float width{12.80f};       // meters
    float height{8.00f};       // meters
    Vec2 gravity{0.0f, -9.81f};
    float globalRestitutionScale{1.0f}; // Multiplier or override support
    bool useGlobalRestitution{false};
    float globalRestitution{0.80f};

    bool airDragEnabled{true};
    bool magnusEnabled{true};
    float airDensity{1.225f};       // kg / m^3
    float magnusLiftFactor{0.08f};  // Spin-curve strength
    bool wallCollisions{true};
    bool ceilingCollisions{true};
    std::size_t subSteps{8};
};

class PhysicsWorld {
public:
    explicit PhysicsWorld(WorldConfig cfg = {});

    Ball& addBall(Vec2 pos, Vec2 vel, float radius, std::size_t materialIdx = 0);
    void clearBalls();
    bool removeBallAt(Vec2 worldPos);
    [[nodiscard]] Ball* findBallAt(Vec2 worldPos) noexcept;
    [[nodiscard]] const Ball* findBallById(std::uint32_t id) const noexcept;
    [[nodiscard]] Ball* findBallById(std::uint32_t id) noexcept;

    void addRamp(Vec2 a, Vec2 b, float thickness = 0.05f);
    bool removeRampAt(Vec2 worldPos);
    void clearRamps();

    // Spring-damper mouse interaction
    void setMouseSpring(bool active, std::uint32_t ballId = 0, Vec2 targetWorldPos = {});

    // Advance physics by dt seconds (internally sub-stepped)
    void step(float dt);

    // Predict trajectory points for slingshot preview
    [[nodiscard]] std::vector<Vec2> predictTrajectory(
        Vec2 startPos,
        Vec2 startVel,
        float radius,
        std::size_t materialIdx,
        float horizonSeconds = 2.2f,
        std::size_t maxPoints = 70) const;

    [[nodiscard]] EnergySnapshot computeSystemEnergy() const noexcept;

    [[nodiscard]] WorldConfig& config() noexcept { return cfg_; }
    [[nodiscard]] const WorldConfig& config() const noexcept { return cfg_; }

    [[nodiscard]] const std::vector<Ball>& balls() const noexcept { return balls_; }
    [[nodiscard]] std::vector<Ball>& balls() noexcept { return balls_; }

    [[nodiscard]] const std::vector<Ramp>& ramps() const noexcept { return ramps_; }

    // Consume impact events generated during the last step()
    [[nodiscard]] std::vector<ImpactEvent> consumeImpacts();

private:
    void stepSub(float dt);
    [[nodiscard]] Vec2 computeAerodynamicAccel(const Ball& b) const noexcept;
    [[nodiscard]] float effectiveRestitution(float baseE) const noexcept;
    void resolveBoundaryCollisions(Ball& b, Vec2 prevPos, Vec2 prevVel, Vec2 accel, float dt);
    void resolveRampCollision(Ball& b, const Ramp& ramp);
    void resolveBallPairCollision(Ball& a, Ball& b);
    void triggerSquash(Ball& b, Vec2 normal, float normalSpeed);
    void recordImpact(Vec2 pos, Vec2 normal, float normalSpeed, float impulse, float radius, Rgba color);

    WorldConfig cfg_{};
    std::vector<Ball> balls_{};
    std::vector<Ramp> ramps_{};
    std::vector<ImpactEvent> impacts_{};
    std::uint32_t nextId_{1};

    bool springActive_{false};
    std::uint32_t springBallId_{0};
    Vec2 springTarget_{};
};

} // namespace bouncy
