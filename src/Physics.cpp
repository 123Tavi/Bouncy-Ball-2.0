#include "Physics.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <utility>

namespace bouncy {

namespace {

constexpr float kPi = std::numbers::pi_v<float>;
constexpr float kRestSpeedThreshold = 0.14f; // m/s threshold for resting contact
constexpr std::size_t kMaxTrailPoints = 55;
constexpr float kTrailSampleInterval = 1.0f / 90.0f;

[[nodiscard]] Vec2 closestPointOnSegment(Vec2 p, Vec2 a, Vec2 b) noexcept {
    const Vec2 ab = b - a;
    const float lenSq = ab.lengthSq();
    if (lenSq <= 1e-10f) {
        return a;
    }
    const float t = std::clamp((p - a).dot(ab) / lenSq, 0.0f, 1.0f);
    return a + ab * t;
}

} // namespace

PhysicsWorld::PhysicsWorld(WorldConfig cfg) : cfg_(cfg) {
    balls_.reserve(64);
}

Ball& PhysicsWorld::addBall(Vec2 pos, Vec2 vel, float radius, std::size_t materialIdx) {
    const auto& mat = kMaterials[materialIdx % kMaterials.size()];
    radius = std::clamp(radius, 0.05f, 1.20f);

    Ball b{};
    b.id = nextId_++;
    b.pos = pos;
    b.vel = vel;
    b.radius = radius;
    b.mass = std::max(0.05f, mat.density * kPi * radius * radius);
    b.inertia = 0.5f * b.mass * radius * radius;
    b.restitution = mat.restitution;
    b.friction = mat.friction;
    b.dragCoeff = mat.dragCoeff;
    b.materialIdx = materialIdx % kMaterials.size();
    b.color = mat.color;

    const float h = std::max(0.0f, pos.y - radius);
    b.initialHeight = h;
    b.currentCyclePeak = h;

    balls_.push_back(std::move(b));
    return balls_.back();
}

void PhysicsWorld::clearBalls() {
    balls_.clear();
    springActive_ = false;
}

bool PhysicsWorld::removeBallAt(Vec2 worldPos) {
    for (auto it = balls_.rbegin(); it != balls_.rend(); ++it) {
        if ((it->pos - worldPos).lengthSq() <= it->radius * it->radius) {
            if (springActive_ && springBallId_ == it->id) {
                springActive_ = false;
            }
            balls_.erase(std::next(it).base());
            return true;
        }
    }
    return false;
}

Ball* PhysicsWorld::findBallAt(Vec2 worldPos) noexcept {
    for (auto it = balls_.rbegin(); it != balls_.rend(); ++it) {
        if ((it->pos - worldPos).lengthSq() <= it->radius * it->radius) {
            return &(*it);
        }
    }
    return nullptr;
}

const Ball* PhysicsWorld::findBallById(std::uint32_t id) const noexcept {
    for (const auto& b : balls_) {
        if (b.id == id) {
            return &b;
        }
    }
    return nullptr;
}

Ball* PhysicsWorld::findBallById(std::uint32_t id) noexcept {
    for (auto& b : balls_) {
        if (b.id == id) {
            return &b;
        }
    }
    return nullptr;
}

void PhysicsWorld::addRamp(Vec2 a, Vec2 b, float thickness) {
    if ((b - a).lengthSq() < 0.01f) {
        return;
    }
    ramps_.push_back(Ramp{
        .a = a,
        .b = b,
        .thickness = thickness,
        .restitution = 0.85f,
        .friction = 0.35f,
    });
}

bool PhysicsWorld::removeRampAt(Vec2 worldPos) {
    for (auto it = ramps_.rbegin(); it != ramps_.rend(); ++it) {
        const Vec2 cp = closestPointOnSegment(worldPos, it->a, it->b);
        const float hitR = std::max(0.14f, it->thickness * 2.2f);
        if ((worldPos - cp).lengthSq() <= hitR * hitR) {
            ramps_.erase(std::next(it).base());
            return true;
        }
    }
    return false;
}

void PhysicsWorld::clearRamps() {
    ramps_.clear();
}

void PhysicsWorld::setMouseSpring(bool active, std::uint32_t ballId, Vec2 targetWorldPos) {
    springActive_ = active;
    springBallId_ = ballId;
    springTarget_ = targetWorldPos;
}

float PhysicsWorld::effectiveRestitution(float baseE) const noexcept {
    if (cfg_.useGlobalRestitution) {
        return std::clamp(cfg_.globalRestitution, 0.0f, 1.05f);
    }
    return std::clamp(baseE * cfg_.globalRestitutionScale, 0.0f, 1.05f);
}

Vec2 PhysicsWorld::computeAerodynamicAccel(const Ball& b) const noexcept {
    Vec2 accel{0.0f, 0.0f};
    const float speedSq = b.vel.lengthSq();
    if (speedSq <= 1e-8f) {
        return accel;
    }

    if (cfg_.airDragEnabled) {
        const float speed = std::sqrt(speedSq);
        // Cross-sectional area scaled softly for 2D visual proportions
        const float area = 2.0f * b.radius * 0.18f;
        const float dragMag = 0.5f * cfg_.airDensity * b.dragCoeff * area * speedSq;
        accel += (-b.vel / speed) * (dragMag / b.mass);
    }

    if (cfg_.magnusEnabled && std::abs(b.angularVel) > 1e-4f) {
        // F_magnus proportional to omega x v -> (-omega * vy, omega * vx)
        const Vec2 magnusDir{-b.angularVel * b.vel.y, b.angularVel * b.vel.x};
        const float liftScale = cfg_.magnusLiftFactor * cfg_.airDensity * b.radius * b.radius * b.radius;
        accel += magnusDir * (liftScale / b.mass);
    }

    return accel;
}

void PhysicsWorld::step(float dt) {
    if (dt <= 0.0f) {
        return;
    }
    dt = std::min(dt, 0.05f); // Guard against window drag stalls

    const std::size_t nSub = std::max<std::size_t>(1, cfg_.subSteps);
    const float subDt = dt / static_cast<float>(nSub);
    for (std::size_t i = 0; i < nSub; ++i) {
        stepSub(subDt);
    }

    // Update trails & squash-and-stretch spring once per frame
    for (auto& b : balls_) {
        b.trailSampleTimer += dt;
        if (b.trailSampleTimer >= kTrailSampleInterval) {
            b.trailSampleTimer = 0.0f;
            b.trail.push_back(TrailPoint{b.pos, b.vel.length()});
            while (b.trail.size() > kMaxTrailPoints) {
                b.trail.pop_front();
            }
        }

        // Critically-damped-ish spring restoring squash back to 0
        constexpr float kStiffness = 260.0f;
        constexpr float kDamping = 22.0f;
        const float squashAccel = -kStiffness * b.squash - kDamping * b.squashVel;
        b.squashVel += squashAccel * dt;
        b.squash += b.squashVel * dt;
        if (std::abs(b.squash) < 1e-4f && std::abs(b.squashVel) < 1e-3f) {
            b.squash = 0.0f;
            b.squashVel = 0.0f;
        } else {
            b.squash = std::clamp(b.squash, -0.22f, 0.38f);
        }
    }
}

void PhysicsWorld::stepSub(float dt) {
    for (auto& b : balls_) {
        const Vec2 prevPos = b.pos;
        const Vec2 prevVel = b.vel;

        Vec2 accel = cfg_.gravity + computeAerodynamicAccel(b);

        if (springActive_ && b.id == springBallId_) {
            constexpr float kSpringFreq = 18.0f;
            constexpr float kSpringDamp = 9.5f;
            const Vec2 delta = springTarget_ - b.pos;
            accel += delta * (kSpringFreq * kSpringFreq) - b.vel * kSpringDamp;
        }

        // Velocity Verlet / midpoint integration before collision check
        b.vel += accel * dt;
        b.pos += (prevVel + b.vel) * (0.5f * dt);

        if (cfg_.airDragEnabled) {
            b.angularVel *= std::max(0.0f, 1.0f - 0.12f * dt);
        }
        b.angle += b.angularVel * dt;

        // Track apex height above floor contact
        const float h = b.heightAboveFloor();
        if (h > b.currentCyclePeak) {
            b.currentCyclePeak = h;
        }

        resolveBoundaryCollisions(b, prevPos, prevVel, accel, dt);

        for (const auto& ramp : ramps_) {
            resolveRampCollision(b, ramp);
        }
    }

    // Pairwise ball-ball collisions (2 solver iterations for crisp multi-ball stacks)
    const std::size_t count = balls_.size();
    for (int iter = 0; iter < 2; ++iter) {
        for (std::size_t i = 0; i < count; ++i) {
            for (std::size_t j = i + 1; j < count; ++j) {
                resolveBallPairCollision(balls_[i], balls_[j]);
            }
        }
    }
}

void PhysicsWorld::resolveBoundaryCollisions(Ball& b, Vec2 prevPos, Vec2 prevVel, Vec2 accel, float dt) {
    const float e = effectiveRestitution(b.restitution);
    const float mu = b.friction;

    auto applyTangentialSpinFriction = [&](Vec2 normal, float normalImpulseMag) {
        if (mu <= 0.0f || normalImpulseMag <= 0.0f) {
            return;
        }
        // Tangent t = normal rotated -90 deg so for floor normal (0, 1), t = (1, 0):
        // Contact point relative to center is r_c = -normal * R.
        // Contact slip velocity: v_contact = v + omega x r_c
        // In 2D: omega x r_c = (-omega * r_cy, omega * r_cx).
        const Vec2 rc = -normal * b.radius;
        const Vec2 omegaCrossR{-b.angularVel * rc.y, b.angularVel * rc.x};
        const Vec2 vContact = b.vel + omegaCrossR;
        const Vec2 tangent{normal.y, -normal.x};
        const float vt = vContact.dot(tangent);
        if (std::abs(vt) <= 1e-6f) {
            return;
        }

        // Effective mass at contact along tangent for solid disk: 1 / (1/m + R^2/I) = 1 / (3/m) = m/3
        const float invEffMass = (1.0f / b.mass) + (b.radius * b.radius) / b.inertia;
        float jt = -vt / invEffMass;
        const float maxJt = mu * normalImpulseMag;
        jt = std::clamp(jt, -maxJt, maxJt);

        const Vec2 impulseT = tangent * jt;
        b.vel += impulseT / b.mass;
        b.angularVel += rc.cross(impulseT) / b.inertia;
    };

    // --- Floor collision (y = radius) with exact within-substep Time-Of-Impact ---
    if (b.pos.y < b.radius) {
        const Vec2 normal{0.0f, 1.0f};
        // Estimate fraction alpha in [0, 1] where pos.y crossed radius
        const float dyTotal = b.pos.y - prevPos.y;
        float alpha = 1.0f;
        if (prevPos.y >= b.radius && dyTotal < -1e-7f) {
            alpha = std::clamp((b.radius - prevPos.y) / dyTotal, 0.0f, 1.0f);
        }

        const float dtBefore = alpha * dt;
        const float dtAfter = (1.0f - alpha) * dt;
        const float vyImpact = prevVel.y + accel.y * dtBefore;
        const float impactSpeed = -vyImpact;

        b.pos.y = b.radius;

        if (impactSpeed > kRestSpeedThreshold) {
            const float vyRebound = e * impactSpeed;
            const float normalImpulse = b.mass * (vyRebound - vyImpact);
            b.vel.y = vyRebound;

            applyTangentialSpinFriction(normal, normalImpulse);

            // Advance remaining fraction of substep so energy is conserved across discrete frames!
            const float vyEnd = b.vel.y + accel.y * dtAfter;
            b.pos.y = std::max(b.radius, b.radius + 0.5f * (b.vel.y + vyEnd) * dtAfter);
            b.vel.y = vyEnd;

            // Update bounce apex telemetry
            b.prevBounceApex = b.lastBounceApex;
            b.lastBounceApex = b.currentCyclePeak;
            if (b.prevBounceApex > 1e-4f) {
                b.lastApexDiff = b.prevBounceApex - b.lastBounceApex;
            }
            const float gMag = std::abs(cfg_.gravity.y);
            b.theoreticalNextApex = (gMag > 1e-4f) ? (vyRebound * vyRebound) / (2.0f * gMag) : 0.0f;
            b.currentCyclePeak = b.heightAboveFloor();
            ++b.bounceCount;

            triggerSquash(b, normal, impactSpeed);
            recordImpact({b.pos.x, 0.0f}, normal, impactSpeed, normalImpulse, b.radius, b.color);
        } else {
            // Resting contact on floor + rolling friction
            const float normalForceImpulse = b.mass * std::max(0.0f, -accel.y) * dt;
            b.vel.y = 0.0f;
            applyTangentialSpinFriction(normal, normalForceImpulse * 0.35f);
            b.vel.x *= std::max(0.0f, 1.0f - 0.25f * mu * dt);
        }
    }

    // --- Ceiling collision (y = height - radius) ---
    if (cfg_.ceilingCollisions && b.pos.y + b.radius > cfg_.height) {
        const Vec2 normal{0.0f, -1.0f};
        const float impactSpeed = b.vel.y;
        b.pos.y = cfg_.height - b.radius;
        if (impactSpeed > 0.0f) {
            const float rebound = (impactSpeed > kRestSpeedThreshold) ? (-e * impactSpeed) : 0.0f;
            const float jn = b.mass * (b.vel.y - rebound);
            b.vel.y = rebound;
            applyTangentialSpinFriction(normal, jn);
            if (impactSpeed > kRestSpeedThreshold) {
                triggerSquash(b, normal, impactSpeed);
                recordImpact({b.pos.x, cfg_.height}, normal, impactSpeed, jn, b.radius, b.color);
            }
        }
    }

    if (!cfg_.wallCollisions) {
        return;
    }

    // --- Left wall (x = radius) ---
    if (b.pos.x < b.radius) {
        const Vec2 normal{1.0f, 0.0f};
        const float impactSpeed = -b.vel.x;
        b.pos.x = b.radius;
        if (impactSpeed > 0.0f) {
            const float rebound = (impactSpeed > kRestSpeedThreshold) ? (e * impactSpeed) : 0.0f;
            const float jn = b.mass * (rebound - b.vel.x);
            b.vel.x = rebound;
            applyTangentialSpinFriction(normal, jn);
            if (impactSpeed > kRestSpeedThreshold) {
                triggerSquash(b, normal, impactSpeed);
                recordImpact({0.0f, b.pos.y}, normal, impactSpeed, jn, b.radius, b.color);
            }
        }
    }

    // --- Right wall (x = width - radius) ---
    if (b.pos.x + b.radius > cfg_.width) {
        const Vec2 normal{-1.0f, 0.0f};
        const float impactSpeed = b.vel.x;
        b.pos.x = cfg_.width - b.radius;
        if (impactSpeed > 0.0f) {
            const float rebound = (impactSpeed > kRestSpeedThreshold) ? (-e * impactSpeed) : 0.0f;
            const float jn = b.mass * (b.vel.x - rebound);
            b.vel.x = rebound;
            applyTangentialSpinFriction(normal, jn);
            if (impactSpeed > kRestSpeedThreshold) {
                triggerSquash(b, normal, impactSpeed);
                recordImpact({cfg_.width, b.pos.y}, normal, impactSpeed, jn, b.radius, b.color);
            }
        }
    }
}

void PhysicsWorld::resolveRampCollision(Ball& b, const Ramp& ramp) {
    const Vec2 cp = closestPointOnSegment(b.pos, ramp.a, ramp.b);
    const Vec2 diff = b.pos - cp;
    const float distSq = diff.lengthSq();
    const float minDist = b.radius + ramp.thickness;
    if (distSq >= minDist * minDist) {
        return;
    }

    const float dist = std::sqrt(distSq);
    const Vec2 normal = (dist > 1e-6f) ? (diff / dist) : (ramp.b - ramp.a).perp().normalized();
    const float penetration = minDist - dist;
    b.pos += normal * penetration;

    const Vec2 rc = -normal * b.radius;
    const Vec2 omegaCrossR{-b.angularVel * rc.y, b.angularVel * rc.x};
    const Vec2 vContact = b.vel + omegaCrossR;

    const float vn = vContact.dot(normal);
    if (vn >= 0.0f) {
        return;
    }

    const float impactSpeed = -vn;
    const float e = (impactSpeed > kRestSpeedThreshold)
                        ? std::sqrt(effectiveRestitution(b.restitution) * ramp.restitution)
                        : 0.0f;
    const float jn = -(1.0f + e) * vn * b.mass;
    b.vel += normal * (jn / b.mass);

    // Tangential friction impulse
    const Vec2 tangent{normal.y, -normal.x};
    const float vt = vContact.dot(tangent);
    const float mu = 0.5f * (b.friction + ramp.friction);
    if (std::abs(vt) > 1e-6f && mu > 0.0f) {
        const float invEffMass = (1.0f / b.mass) + (b.radius * b.radius) / b.inertia;
        const float maxJt = mu * jn;
        const float jt = std::clamp(-vt / invEffMass, -maxJt, maxJt);
        const Vec2 impulseT = tangent * jt;
        b.vel += impulseT / b.mass;
        b.angularVel += rc.cross(impulseT) / b.inertia;
    }

    if (impactSpeed > kRestSpeedThreshold) {
        ++b.bounceCount;
        triggerSquash(b, normal, impactSpeed);
        recordImpact(cp, normal, impactSpeed, jn, b.radius, b.color);
    }
}

void PhysicsWorld::resolveBallPairCollision(Ball& a, Ball& b) {
    const Vec2 delta = b.pos - a.pos;
    const float minDist = a.radius + b.radius;
    const float distSq = delta.lengthSq();
    if (distSq >= minDist * minDist) {
        return;
    }

    const float dist = std::sqrt(distSq);
    const Vec2 normal = (dist > 1e-6f) ? (delta / dist) : Vec2{1.0f, 0.0f};
    const float penetration = minDist - dist;

    const float invMassA = 1.0f / a.mass;
    const float invMassB = 1.0f / b.mass;
    const float invMassSum = invMassA + invMassB;

    // Positional correction weighted by inverse mass
    constexpr float kSlop = 0.0005f;
    constexpr float kPercent = 0.85f;
    const float corrMag = std::max(0.0f, penetration - kSlop) * kPercent / invMassSum;
    a.pos -= normal * (corrMag * invMassA);
    b.pos += normal * (corrMag * invMassB);

    const Vec2 ra = normal * a.radius;
    const Vec2 rb = -normal * b.radius;
    const Vec2 vaContact = a.vel + Vec2{-a.angularVel * ra.y, a.angularVel * ra.x};
    const Vec2 vbContact = b.vel + Vec2{-b.angularVel * rb.y, b.angularVel * rb.x};
    const Vec2 relVel = vbContact - vaContact;

    const float vn = relVel.dot(normal);
    if (vn >= 0.0f) {
        return; // Already separating
    }

    const float impactSpeed = -vn;
    const float eA = effectiveRestitution(a.restitution);
    const float eB = effectiveRestitution(b.restitution);
    const float e = (impactSpeed > kRestSpeedThreshold) ? std::sqrt(eA * eB) : 0.0f;

    const float jn = -(1.0f + e) * vn / invMassSum;
    const Vec2 impulseN = normal * jn;
    a.vel -= impulseN * invMassA;
    b.vel += impulseN * invMassB;

    // Coulomb tangential impulse transferring spin between colliding balls
    const Vec2 tangent{-normal.y, normal.x};
    const float vt = relVel.dot(tangent);
    const float mu = std::sqrt(a.friction * b.friction);
    if (std::abs(vt) > 1e-6f && mu > 0.0f) {
        const float raCrossT = ra.cross(tangent);
        const float rbCrossT = rb.cross(tangent);
        const float invEffTangent = invMassSum
                                  + (raCrossT * raCrossT) / a.inertia
                                  + (rbCrossT * rbCrossT) / b.inertia;
        const float maxJt = mu * jn;
        const float jt = std::clamp(-vt / invEffTangent, -maxJt, maxJt);
        const Vec2 impulseT = tangent * jt;

        a.vel -= impulseT * invMassA;
        b.vel += impulseT * invMassB;
        a.angularVel -= ra.cross(impulseT) / a.inertia;
        b.angularVel += rb.cross(impulseT) / b.inertia;
    }

    if (impactSpeed > kRestSpeedThreshold) {
        triggerSquash(a, -normal, impactSpeed * (b.mass / (a.mass + b.mass)) * 1.6f);
        triggerSquash(b, normal, impactSpeed * (a.mass / (a.mass + b.mass)) * 1.6f);
        const Vec2 contactPt = a.pos + normal * a.radius;
        recordImpact(contactPt, normal, impactSpeed, jn, std::min(a.radius, b.radius), a.color);
    }
}

void PhysicsWorld::triggerSquash(Ball& b, Vec2 normal, float normalSpeed) {
    const float target = std::clamp(normalSpeed * 0.032f, 0.0f, 0.34f);
    if (target > b.squash) {
        b.squashNormal = normal.normalized();
        b.squash = target;
        b.squashVel = normalSpeed * 0.45f;
    }
}

void PhysicsWorld::recordImpact(Vec2 pos, Vec2 normal, float normalSpeed, float impulse, float radius, Rgba color) {
    if (impacts_.size() < 64 && normalSpeed > 0.35f) {
        impacts_.push_back(ImpactEvent{
            .pos = pos,
            .normal = normal,
            .normalSpeed = normalSpeed,
            .impulse = impulse,
            .radius = radius,
            .color = color,
        });
    }
}

std::vector<ImpactEvent> PhysicsWorld::consumeImpacts() {
    std::vector<ImpactEvent> out{};
    out.swap(impacts_);
    return out;
}

std::vector<Vec2> PhysicsWorld::predictTrajectory(
    Vec2 startPos,
    Vec2 startVel,
    float radius,
    std::size_t materialIdx,
    float horizonSeconds,
    std::size_t maxPoints) const {
    PhysicsWorld previewWorld(cfg_);
    previewWorld.config().subSteps = 2;
    for (const auto& ramp : ramps_) {
        previewWorld.addRamp(ramp.a, ramp.b, ramp.thickness);
    }
    previewWorld.addBall(startPos, startVel, radius, materialIdx);

    std::vector<Vec2> pts;
    pts.reserve(maxPoints);
    if (maxPoints == 0) {
        return pts;
    }

    const float dt = horizonSeconds / static_cast<float>(maxPoints);
    for (std::size_t i = 0; i < maxPoints; ++i) {
        pts.push_back(previewWorld.balls().front().pos);
        previewWorld.step(dt);
    }
    return pts;
}

EnergySnapshot PhysicsWorld::computeSystemEnergy() const noexcept {
    EnergySnapshot snap{};
    const float gMag = cfg_.gravity.length();
    for (const auto& b : balls_) {
        snap.translationalKE += b.translationalKE();
        snap.rotationalKE += b.rotationalKE();
        snap.potential += b.potentialEnergy(gMag);
    }
    snap.total = snap.translationalKE + snap.rotationalKE + snap.potential;
    return snap;
}

} // namespace bouncy
