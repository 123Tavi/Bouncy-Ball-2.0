#include "AudioSynth.hpp"
#include "Physics.hpp"

#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <iomanip>
#include <numbers>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

using bouncy::Ball;
using bouncy::EnergySnapshot;
using bouncy::ImpactEvent;
using bouncy::kMaterials;
using bouncy::PhysicsWorld;
using bouncy::Rgba;
using bouncy::Vec2;
using bouncy::WorldConfig;

constexpr float kPixelsPerMeter = 100.0f;
constexpr float kFloorMarginPx = 44.0f;
constexpr float kLeftMarginPx = 54.0f;
constexpr unsigned int kInitWinW = 1280;
constexpr unsigned int kInitWinH = 800;

struct GravityPreset {
    const char* name;
    Vec2 g;
};

constexpr std::array<GravityPreset, 5> kGravityPresets{{
    {"Earth (9.81 m/s^2)",   {0.0f,  -9.81f}},
    {"Moon (1.62 m/s^2)",    {0.0f,  -1.62f}},
    {"Mars (3.72 m/s^2)",    {0.0f,  -3.72f}},
    {"Jupiter (24.79 m/s^2)",{0.0f, -24.79f}},
    {"Zero-G (0.00 m/s^2)",  {0.0f,   0.00f}},
}};

struct SparkParticle {
    Vec2 pos{};
    Vec2 vel{};
    float life{0.0f};
    float maxLife{0.4f};
    Rgba color{};
};

[[nodiscard]] sf::Color toSfColor(Rgba c, std::uint8_t alphaOverride = 255, bool useOverride = false) {
    return sf::Color{c.r, c.g, c.b, useOverride ? alphaOverride : c.a};
}

[[nodiscard]] sf::Color scaleColor(Rgba c, float factor, std::uint8_t alpha = 255) {
    const auto ch = [factor](std::uint8_t v) -> std::uint8_t {
        return static_cast<std::uint8_t>(std::clamp(static_cast<float>(v) * factor, 0.0f, 255.0f));
    };
    return sf::Color{ch(c.r), ch(c.g), ch(c.b), alpha};
}

struct ScreenMapper {
    float winW{static_cast<float>(kInitWinW)};
    float winH{static_cast<float>(kInitWinH)};

    [[nodiscard]] float worldWidth() const noexcept {
        return std::max(3.0f, (winW - kLeftMarginPx - 20.0f) / kPixelsPerMeter);
    }

    [[nodiscard]] float worldHeight() const noexcept {
        return std::max(2.5f, (winH - kFloorMarginPx - 20.0f) / kPixelsPerMeter);
    }

    [[nodiscard]] sf::Vector2f toScreen(Vec2 w) const noexcept {
        return {
            kLeftMarginPx + w.x * kPixelsPerMeter,
            (winH - kFloorMarginPx) - w.y * kPixelsPerMeter,
        };
    }

    [[nodiscard]] Vec2 toWorld(sf::Vector2f s) const noexcept {
        return {
            (s.x - kLeftMarginPx) / kPixelsPerMeter,
            ((winH - kFloorMarginPx) - s.y) / kPixelsPerMeter,
        };
    }
};

bool loadSystemFont(sf::Font& font) {
    const std::array<const char*, 5> candidates{{
        "/System/Library/Fonts/SFNSMono.ttf",
        "/System/Library/Fonts/Menlo.ttc",
        "/System/Library/Fonts/Monaco.ttf",
        "/System/Library/Fonts/Courier.ttc",
        "/System/Library/Fonts/Helvetica.ttc",
    }};
    for (const char* path : candidates) {
        if (std::filesystem::exists(path) && font.openFromFile(path)) {
            return true;
        }
    }
    return false;
}

void drawLine(sf::RenderTarget& target, sf::Vector2f a, sf::Vector2f b, sf::Color ca, sf::Color cb) {
    const std::array<sf::Vertex, 2> verts{{
        sf::Vertex{a, ca, {}},
        sf::Vertex{b, cb, {}},
    }};
    target.draw(verts.data(), verts.size(), sf::PrimitiveType::Lines);
}

void drawThickSegment(sf::RenderTarget& target, sf::Vector2f a, sf::Vector2f b, float halfThickPx, sf::Color col) {
    const sf::Vector2f d = b - a;
    const float len = std::hypot(d.x, d.y);
    if (len <= 1e-4f) {
        return;
    }
    const sf::Vector2f n{-d.y / len * halfThickPx, d.x / len * halfThickPx};
    const std::array<sf::Vertex, 4> quad{{
        sf::Vertex{a + n, col, {}},
        sf::Vertex{a - n, col, {}},
        sf::Vertex{b + n, col, {}},
        sf::Vertex{b - n, col, {}},
    }};
    target.draw(quad.data(), quad.size(), sf::PrimitiveType::TriangleStrip);
}

void drawDashedHLine(sf::RenderTarget& target, float x0, float x1, float y, sf::Color col, float dashPx = 8.0f) {
    for (float x = x0; x < x1; x += dashPx * 2.0f) {
        drawLine(target, {x, y}, {std::min(x + dashPx, x1), y}, col, col);
    }
}

void drawArrow(sf::RenderTarget& target, sf::Vector2f from, sf::Vector2f to, sf::Color col) {
    const sf::Vector2f d = to - from;
    const float len = std::hypot(d.x, d.y);
    if (len < 3.0f) {
        return;
    }
    drawThickSegment(target, from, to, 1.4f, col);
    const sf::Vector2f u{d.x / len, d.y / len};
    const sf::Vector2f p{-u.y, u.x};
    const float head = std::min(9.0f, len * 0.35f);
    const sf::Vector2f left = to - u * head + p * (head * 0.55f);
    const sf::Vector2f right = to - u * head - p * (head * 0.55f);
    const std::array<sf::Vertex, 3> tri{{
        sf::Vertex{to, col, {}},
        sf::Vertex{left, col, {}},
        sf::Vertex{right, col, {}},
    }};
    target.draw(tri.data(), tri.size(), sf::PrimitiveType::Triangles);
}

void drawShadedBall(sf::RenderTarget& target, const ScreenMapper& mapper, const Ball& b, bool isSelected) {
    const sf::Vector2f center = mapper.toScreen(b.pos);
    const float rPx = b.radius * kPixelsPerMeter;

    // Squash along collision normal while preserving constant 2D area (scaleN * scaleT = 1)
    const float scaleN = std::clamp(1.0f - b.squash, 0.62f, 1.25f);
    const float scaleT = 1.0f / scaleN;
    // Screen normal: world +y is screen -y
    const sf::Vector2f nScreen{b.squashNormal.x, -b.squashNormal.y};
    const float normalAngleRad = std::atan2(nScreen.y, nScreen.x);

    // Outer transform: move to center -> align squash normal -> scale (scaleN, scaleT) -> unrotate
    sf::Transform squashXf;
    squashXf.translate(center);
    squashXf.rotate(sf::radians(normalAngleRad));
    squashXf.scale({scaleN, scaleT});
    squashXf.rotate(sf::radians(-normalAngleRad));

    sf::RenderStates states{};
    states.transform = squashXf;

    // Selection halo
    if (isSelected) {
        sf::CircleShape halo(rPx + 4.5f, 44);
        halo.setOrigin({rPx + 4.5f, rPx + 4.5f});
        halo.setFillColor(sf::Color::Transparent);
        halo.setOutlineThickness(2.0f);
        halo.setOutlineColor(sf::Color(255, 255, 255, 190));
        target.draw(halo, states);
    }

    // Soft drop shadow near floor
    if (b.heightAboveFloor() < 2.2f) {
        const float factor = std::clamp(1.0f - b.heightAboveFloor() / 2.2f, 0.0f, 1.0f);
        const sf::Vector2f floorPt = mapper.toScreen({b.pos.x, 0.0f});
        sf::CircleShape shadow(rPx, 32);
        shadow.setOrigin({rPx, rPx});
        shadow.setPosition({floorPt.x, floorPt.y + 3.0f});
        shadow.setScale({1.05f * (0.55f + 0.45f * factor) * scaleT, 0.22f});
        shadow.setFillColor(sf::Color(0, 0, 0, static_cast<std::uint8_t>(120.0f * factor)));
        target.draw(shadow);
    }

    // Base body circle (gradient-like radial fan centered slightly upper-left)
    constexpr int kSegs = 44;
    std::array<sf::Vertex, kSegs + 2> bodyFan{};
    bodyFan[0] = sf::Vertex{{-rPx * 0.25f, -rPx * 0.28f}, scaleColor(b.color, 1.28f), {}};
    const sf::Color rimCol = scaleColor(b.color, 0.62f);
    for (int i = 0; i <= kSegs; ++i) {
        const float a = static_cast<float>(i) / static_cast<float>(kSegs) * 2.0f * std::numbers::pi_v<float>;
        bodyFan[static_cast<std::size_t>(i + 1)] = sf::Vertex{
            {std::cos(a) * rPx, std::sin(a) * rPx},
            rimCol,
            {},
        };
    }
    target.draw(bodyFan.data(), bodyFan.size(), sf::PrimitiveType::TriangleFan, states);

    // Rotation stripe & rim markers (world +angle = CCW -> screen -angle)
    sf::Transform spinXf = squashXf;
    spinXf.rotate(sf::radians(-b.angle));
    sf::RenderStates spinStates{};
    spinStates.transform = spinXf;

    sf::RectangleShape stripe({rPx * 1.72f, std::max(2.2f, rPx * 0.14f)});
    stripe.setOrigin({rPx * 0.86f, std::max(1.1f, rPx * 0.07f)});
    stripe.setFillColor(sf::Color(255, 255, 255, 95));
    target.draw(stripe, spinStates);

    sf::CircleShape markerDot(std::max(2.0f, rPx * 0.13f), 14);
    markerDot.setOrigin({markerDot.getRadius(), markerDot.getRadius()});
    markerDot.setPosition({rPx * 0.62f, 0.0f});
    markerDot.setFillColor(sf::Color(255, 255, 255, 195));
    target.draw(markerDot, spinStates);

    // Crisp outline + specular shine
    sf::CircleShape outline(rPx - 1.0f, kSegs);
    outline.setOrigin({rPx - 1.0f, rPx - 1.0f});
    outline.setFillColor(sf::Color::Transparent);
    outline.setOutlineThickness(1.5f);
    outline.setOutlineColor(scaleColor(b.color, 1.35f, 220));
    target.draw(outline, states);

    sf::CircleShape spec(rPx * 0.24f, 18);
    spec.setOrigin({rPx * 0.24f, rPx * 0.24f});
    spec.setPosition({-rPx * 0.32f, -rPx * 0.34f});
    spec.setRotation(sf::degrees(-32.0f));
    spec.setScale({1.25f, 0.72f});
    spec.setFillColor(sf::Color(255, 255, 255, 95));
    target.draw(spec, states);
}

std::string fmtFloat(float v, int prec = 2) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(prec) << v;
    return oss.str();
}

} // namespace

int main(int argc, char* argv[]) {
    const bool smokeTest = (argc > 1 && std::string_view(argv[1]) == "--smoke-test");

    sf::RenderWindow window(
        sf::VideoMode({kInitWinW, kInitWinH}),
        "Bouncy-Ball 3.0 - C++20 & SFML 2D Physics Lab",
        sf::Style::Default,
        sf::State::Windowed);
    window.setVerticalSyncEnabled(true);

    sf::Font font;
    const bool hasFont = loadSystemFont(font);

    ScreenMapper mapper{};
    PhysicsWorld world(WorldConfig{
        .width = mapper.worldWidth(),
        .height = mapper.worldHeight(),
    });

    bouncy::AudioSynth audio{};
    std::mt19937 rng{42u};

    std::uint32_t selectedBallId = 0;
    std::size_t activeMaterial = 0;
    std::size_t gravityIdx = 0;
    float spawnRadius = 0.22f;
    float spawnSpin = 0.0f;

    bool paused = false;
    bool stepSingleFrame = false;
    bool showGrid = true;
    bool showTrails = true;
    bool showVectors = false;
    bool showHelp = true;
    bool showApexGuides = true;

    // Mouse interaction state
    bool slingshotActive = false;
    Vec2 slingshotAnchorWorld{};
    bool grabActive = false;
    std::uint32_t grabbedBallId = 0;
    bool rampDrawActive = false;
    Vec2 rampStartWorld{};

    std::vector<SparkParticle> sparks{};
    std::deque<EnergySnapshot> energyHistory{};
    constexpr std::size_t kMaxEnergySamples = 240;
    float energySampleAccum = 0.0f;

    auto resetClassicLab = [&]() {
        world.clearBalls();
        world.clearRamps();
        energyHistory.clear();
        sparks.clear();
        world.config().useGlobalRestitution = false;
        world.config().globalRestitutionScale = 1.0f;
        gravityIdx = 0;
        world.config().gravity = kGravityPresets[0].g;

        // Classic 2.0 drop from center of world, upgraded with real units!
        const Vec2 center{mapper.worldWidth() * 0.5f, mapper.worldHeight() * 0.58f};
        Ball& b = world.addBall(center, {0.0f, 0.0f}, 0.22f, 0);
        selectedBallId = b.id;
    };

    auto spawnChaosBurst = [&]() {
        std::uniform_real_distribution<float> rx(0.8f, mapper.worldWidth() - 0.8f);
        std::uniform_real_distribution<float> ry(mapper.worldHeight() * 0.35f, mapper.worldHeight() - 0.7f);
        std::uniform_real_distribution<float> rv(-4.8f, 4.8f);
        std::uniform_real_distribution<float> rr(0.13f, 0.32f);
        std::uniform_real_distribution<float> rw(-10.0f, 10.0f);
        std::uniform_int_distribution<std::size_t> rm(0, kMaterials.size() - 1);

        for (int i = 0; i < 14; ++i) {
            Ball& b = world.addBall({rx(rng), ry(rng)}, {rv(rng), rv(rng) * 0.5f}, rr(rng), rm(rng));
            b.angularVel = rw(rng);
            selectedBallId = b.id;
        }
    };

    resetClassicLab();

    sf::Clock frameClock;
    float fpsSmooth = 60.0f;

    while (window.isOpen()) {
        const sf::Vector2i mousePxInt = sf::Mouse::getPosition(window);
        const sf::Vector2f mouseScreen{static_cast<float>(mousePxInt.x), static_cast<float>(mousePxInt.y)};
        const Vec2 mouseWorld = mapper.toWorld(mouseScreen);

        while (const std::optional<sf::Event> event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) {
                window.close();
            } else if (const auto* resized = event->getIf<sf::Event::Resized>()) {
                mapper.winW = static_cast<float>(resized->size.x);
                mapper.winH = static_cast<float>(resized->size.y);
                window.setView(sf::View(sf::FloatRect({0.0f, 0.0f}, {mapper.winW, mapper.winH})));
                world.config().width = mapper.worldWidth();
                world.config().height = mapper.worldHeight();
            } else if (const auto* wheel = event->getIf<sf::Event::MouseWheelScrolled>()) {
                if (sf::Keyboard::isKeyPressed(sf::Keyboard::Key::LShift) ||
                    sf::Keyboard::isKeyPressed(sf::Keyboard::Key::RShift)) {
                    spawnSpin = std::clamp(spawnSpin + wheel->delta * 2.5f, -35.0f, 35.0f);
                } else {
                    spawnRadius = std::clamp(spawnRadius + wheel->delta * 0.02f, 0.07f, 0.65f);
                }
            } else if (const auto* mbp = event->getIf<sf::Event::MouseButtonPressed>()) {
                const sf::Vector2f clickScreen{static_cast<float>(mbp->position.x), static_cast<float>(mbp->position.y)};
                const Vec2 clickWorld = mapper.toWorld(clickScreen);

                if (mbp->button == sf::Mouse::Button::Left) {
                    if (Ball* hit = world.findBallAt(clickWorld)) {
                        selectedBallId = hit->id;
                        grabActive = true;
                        grabbedBallId = hit->id;
                        world.setMouseSpring(true, grabbedBallId, clickWorld);
                    } else if (clickWorld.x > 0.0f && clickWorld.x < world.config().width &&
                               clickWorld.y > 0.0f && clickWorld.y < world.config().height) {
                        slingshotActive = true;
                        slingshotAnchorWorld = clickWorld;
                    }
                } else if (mbp->button == sf::Mouse::Button::Right) {
                    if (!world.removeBallAt(clickWorld) && !world.removeRampAt(clickWorld)) {
                        rampDrawActive = true;
                        rampStartWorld = clickWorld;
                    }
                }
            } else if (const auto* mbr = event->getIf<sf::Event::MouseButtonReleased>()) {
                const sf::Vector2f relScreen{static_cast<float>(mbr->position.x), static_cast<float>(mbr->position.y)};
                const Vec2 relWorld = mapper.toWorld(relScreen);

                if (mbr->button == sf::Mouse::Button::Left) {
                    if (grabActive) {
                        grabActive = false;
                        world.setMouseSpring(false);
                    }
                    if (slingshotActive) {
                        slingshotActive = false;
                        const Vec2 launchVel = (slingshotAnchorWorld - relWorld) * 3.6f;
                        const Vec2 safePos{
                            std::clamp(slingshotAnchorWorld.x, spawnRadius, world.config().width - spawnRadius),
                            std::clamp(slingshotAnchorWorld.y, spawnRadius, world.config().height - spawnRadius),
                        };
                        Ball& b = world.addBall(safePos, launchVel, spawnRadius, activeMaterial);
                        b.angularVel = spawnSpin;
                        selectedBallId = b.id;
                    }
                } else if (mbr->button == sf::Mouse::Button::Right && rampDrawActive) {
                    rampDrawActive = false;
                    world.addRamp(rampStartWorld, relWorld, 0.045f);
                }
            } else if (const auto* kp = event->getIf<sf::Event::KeyPressed>()) {
                using Key = sf::Keyboard::Key;
                switch (kp->code) {
                    case Key::Escape: window.close(); break;
                    case Key::Space: paused = !paused; break;
                    case Key::S:
                        paused = true;
                        stepSingleFrame = true;
                        break;
                    case Key::R: resetClassicLab(); break;
                    case Key::C:
                        world.clearBalls();
                        energyHistory.clear();
                        break;
                    case Key::X: world.clearRamps(); break;
                    case Key::B: spawnChaosBurst(); break;
                    case Key::G: showGrid = !showGrid; break;
                    case Key::T: showTrails = !showTrails; break;
                    case Key::V: showVectors = !showVectors; break;
                    case Key::A: showApexGuides = !showApexGuides; break;
                    case Key::H: showHelp = !showHelp; break;
                    case Key::M: audio.toggleMute(); break;
                    case Key::D:
                        world.config().airDragEnabled = !world.config().airDragEnabled;
                        world.config().magnusEnabled = world.config().airDragEnabled;
                        break;
                    case Key::LBracket:
                        gravityIdx = (gravityIdx + kGravityPresets.size() - 1) % kGravityPresets.size();
                        world.config().gravity = kGravityPresets[gravityIdx].g;
                        break;
                    case Key::RBracket:
                        gravityIdx = (gravityIdx + 1) % kGravityPresets.size();
                        world.config().gravity = kGravityPresets[gravityIdx].g;
                        break;
                    case Key::Up: {
                        if (!world.config().useGlobalRestitution) {
                            world.config().useGlobalRestitution = true;
                            world.config().globalRestitution = kMaterials[activeMaterial].restitution;
                        }
                        world.config().globalRestitution =
                            std::clamp(world.config().globalRestitution + 0.05f, 0.0f, 1.0f);
                        break;
                    }
                    case Key::Down: {
                        if (!world.config().useGlobalRestitution) {
                            world.config().useGlobalRestitution = true;
                            world.config().globalRestitution = kMaterials[activeMaterial].restitution;
                        }
                        world.config().globalRestitution =
                            std::clamp(world.config().globalRestitution - 0.05f, 0.0f, 1.0f);
                        break;
                    }
                    case Key::P:
                        // Pure conservation test mode: e = 1.0, zero drag, zero friction!
                        world.config().useGlobalRestitution = true;
                        world.config().globalRestitution = 1.0f;
                        world.config().airDragEnabled = false;
                        world.config().magnusEnabled = false;
                        for (auto& b : world.balls()) {
                            b.friction = 0.0f;
                        }
                        break;
                    case Key::Q: spawnSpin = std::clamp(spawnSpin + 4.0f, -35.0f, 35.0f); break;
                    case Key::E: spawnSpin = std::clamp(spawnSpin - 4.0f, -35.0f, 35.0f); break;
                    case Key::Num1: activeMaterial = 0; world.config().useGlobalRestitution = false; break;
                    case Key::Num2: activeMaterial = 1; world.config().useGlobalRestitution = false; break;
                    case Key::Num3: activeMaterial = 2; world.config().useGlobalRestitution = false; break;
                    case Key::Num4: activeMaterial = 3; world.config().useGlobalRestitution = false; break;
                    case Key::Num5: activeMaterial = 4; world.config().useGlobalRestitution = false; break;
                    case Key::Num6: activeMaterial = 5; world.config().useGlobalRestitution = false; break;
                    default: break;
                }
            }
        }

        if (grabActive) {
            world.setMouseSpring(true, grabbedBallId, mouseWorld);
        }

        const float rawDt = frameClock.restart().asSeconds();
        const float dt = std::min(rawDt, 0.033f);
        if (rawDt > 1e-5f) {
            fpsSmooth = fpsSmooth * 0.92f + (1.0f / rawDt) * 0.08f;
        }

        if (!paused || stepSingleFrame) {
            const float simDt = stepSingleFrame ? (1.0f / 60.0f) : dt;
            stepSingleFrame = false;
            world.step(simDt);

            energySampleAccum += simDt;
            if (energySampleAccum >= (1.0f / 60.0f)) {
                energySampleAccum = 0.0f;
                energyHistory.push_back(world.computeSystemEnergy());
                while (energyHistory.size() > kMaxEnergySamples) {
                    energyHistory.pop_front();
                }
            }
        }

        // Process collision impacts -> audio + spark particles
        std::uniform_real_distribution<float> jitter(-1.0f, 1.0f);
        for (const ImpactEvent& ev : world.consumeImpacts()) {
            audio.playImpact(ev.normalSpeed, ev.radius, ev.pos.x / std::max(1.0f, world.config().width));

            const int numSparks = std::clamp(static_cast<int>(ev.normalSpeed * 2.2f), 0, 18);
            const Vec2 tangent{-ev.normal.y, ev.normal.x};
            for (int i = 0; i < numSparks && sparks.size() < 450; ++i) {
                const float spread = jitter(rng) * 0.95f;
                const float speed = (0.35f + 0.45f * std::abs(jitter(rng))) * ev.normalSpeed;
                const Vec2 dir = (ev.normal * (0.45f + 0.55f * std::abs(jitter(rng))) + tangent * spread).normalized();
                sparks.push_back(SparkParticle{
                    .pos = ev.pos + ev.normal * 0.02f,
                    .vel = dir * speed,
                    .life = 0.0f,
                    .maxLife = 0.22f + 0.22f * std::abs(jitter(rng)),
                    .color = ev.color,
                });
            }
        }

        // Update sparks
        for (auto it = sparks.begin(); it != sparks.end();) {
            it->life += dt;
            if (it->life >= it->maxLife) {
                it = sparks.erase(it);
            } else {
                it->vel += world.config().gravity * (0.55f * dt);
                it->pos += it->vel * dt;
                if (it->pos.y < 0.0f) {
                    it->pos.y = 0.0f;
                    it->vel.y *= -0.45f;
                }
                ++it;
            }
        }

        // Ensure selectedBallId points to an existing ball if any exist
        const Ball* selectedBall = world.findBallById(selectedBallId);
        if (!selectedBall && !world.balls().empty()) {
            selectedBallId = world.balls().back().id;
            selectedBall = &world.balls().back();
        }

        // ===================== RENDER =====================
        window.clear(sf::Color(13, 16, 23));

        const sf::Vector2f originScreen = mapper.toScreen({0.0f, 0.0f});
        const sf::Vector2f topRightScreen = mapper.toScreen({world.config().width, world.config().height});

        // Metric coordinate grid (upgraded homage to 2.0's 50px X/Y grid!)
        if (showGrid) {
            constexpr float kMinorStep = 0.25f; // 25 px
            for (float xm = 0.0f; xm <= world.config().width + 1e-3f; xm += kMinorStep) {
                const int idx = static_cast<int>(std::round(xm / kMinorStep));
                const bool isMajor = (idx % 4 == 0);
                const sf::Vector2f p0 = mapper.toScreen({xm, 0.0f});
                const sf::Vector2f p1 = mapper.toScreen({xm, world.config().height});
                const sf::Color col = isMajor ? sf::Color(48, 58, 76, 175) : sf::Color(26, 32, 44, 130);
                drawLine(window, p0, p1, col, col);

                if (hasFont && isMajor) {
                    sf::Text lbl(font, std::to_string(idx / 4) + "m", 11);
                    lbl.setFillColor(sf::Color(135, 150, 175));
                    lbl.setPosition({p0.x - 6.0f, originScreen.y + 8.0f});
                    window.draw(lbl);
                }
            }

            for (float ym = 0.0f; ym <= world.config().height + 1e-3f; ym += kMinorStep) {
                const int idx = static_cast<int>(std::round(ym / kMinorStep));
                const bool isMajor = (idx % 4 == 0);
                const sf::Vector2f p0 = mapper.toScreen({0.0f, ym});
                const sf::Vector2f p1 = mapper.toScreen({world.config().width, ym});
                const sf::Color col = isMajor ? sf::Color(48, 58, 76, 175) : sf::Color(26, 32, 44, 130);
                drawLine(window, p0, p1, col, col);

                if (hasFont && isMajor) {
                    sf::Text lbl(font, std::to_string(idx / 4) + "m", 11);
                    lbl.setFillColor(sf::Color(135, 150, 175));
                    lbl.setPosition({10.0f, p0.y - 8.0f});
                    window.draw(lbl);
                }
            }
        }

        // Arena boundary & highlighted X/Y axes
        sf::RectangleShape floorFill({topRightScreen.x - originScreen.x, mapper.winH - originScreen.y});
        floorFill.setPosition({originScreen.x, originScreen.y});
        floorFill.setFillColor(sf::Color(20, 25, 36));
        window.draw(floorFill);

        drawThickSegment(window, originScreen, {topRightScreen.x, originScreen.y}, 1.8f, sf::Color(210, 222, 240));
        drawThickSegment(window, originScreen, {originScreen.x, topRightScreen.y}, 1.8f, sf::Color(210, 222, 240));
        drawLine(window, {topRightScreen.x, originScreen.y}, topRightScreen, sf::Color(90, 105, 130), sf::Color(90, 105, 130));
        drawLine(window, {originScreen.x, topRightScreen.y}, topRightScreen, sf::Color(90, 105, 130), sf::Color(90, 105, 130));

        // Apex height guides for selected ball (completes 2.0's `// TODO GET FINAL HEIGHT`)
        if (showApexGuides && selectedBall) {
            auto drawHeightMarker = [&](float hAboveFloor, sf::Color col, const std::string& tag) {
                if (hAboveFloor <= 0.02f || hAboveFloor > world.config().height) {
                    return;
                }
                const float worldY = hAboveFloor + selectedBall->radius;
                const float sy = mapper.toScreen({0.0f, worldY}).y;
                drawDashedHLine(window, originScreen.x, topRightScreen.x, sy, col, 7.0f);
                if (hasFont) {
                    sf::Text t(font, tag + " (" + fmtFloat(hAboveFloor, 2) + " m)", 11);
                    t.setFillColor(col);
                    t.setPosition({originScreen.x + 8.0f, sy - 16.0f});
                    window.draw(t);
                }
            };

            drawHeightMarker(selectedBall->initialHeight, sf::Color(120, 165, 255, 120), "Initial Drop h0");
            if (selectedBall->lastBounceApex > 0.02f) {
                drawHeightMarker(selectedBall->lastBounceApex, sf::Color(90, 245, 175, 165), "Last Bounce Apex");
            }
        }

        // Draw static ramps
        for (const auto& ramp : world.ramps()) {
            const sf::Vector2f a = mapper.toScreen(ramp.a);
            const sf::Vector2f b = mapper.toScreen(ramp.b);
            const float rPx = ramp.thickness * kPixelsPerMeter;
            drawThickSegment(window, a, b, rPx, sf::Color(156, 176, 206));
            sf::CircleShape cap(rPx, 16);
            cap.setOrigin({rPx, rPx});
            cap.setFillColor(sf::Color(156, 176, 206));
            cap.setPosition(a);
            window.draw(cap);
            cap.setPosition(b);
            window.draw(cap);
        }

        if (rampDrawActive) {
            drawThickSegment(
                window,
                mapper.toScreen(rampStartWorld),
                mouseScreen,
                0.045f * kPixelsPerMeter,
                sf::Color(156, 176, 206, 140));
        }

        // Motion trails
        if (showTrails) {
            for (const auto& b : world.balls()) {
                if (b.trail.size() < 2) {
                    continue;
                }
                for (std::size_t i = 1; i < b.trail.size(); ++i) {
                    const float frac = static_cast<float>(i) / static_cast<float>(b.trail.size());
                    const auto alpha = static_cast<std::uint8_t>(frac * frac * 155.0f);
                    const float width = std::max(1.0f, b.radius * kPixelsPerMeter * 0.36f * frac);
                    drawThickSegment(
                        window,
                        mapper.toScreen(b.trail[i - 1].pos),
                        mapper.toScreen(b.trail[i].pos),
                        width,
                        toSfColor(b.color, alpha, true));
                }
            }
        }

        // Draw balls & optional velocity/gravity vectors
        for (const auto& b : world.balls()) {
            drawShadedBall(window, mapper, b, b.id == selectedBallId);

            if (showVectors) {
                const sf::Vector2f c = mapper.toScreen(b.pos);
                // Velocity vector (green)
                const sf::Vector2f vTip = mapper.toScreen(b.pos + b.vel * 0.16f);
                drawArrow(window, c, vTip, sf::Color(80, 245, 145));
                // Gravity vector (amber)
                const sf::Vector2f gTip = mapper.toScreen(b.pos + world.config().gravity * 0.05f);
                drawArrow(window, c, gTip, sf::Color(255, 195, 75, 200));
            }
        }

        // Spring grab line
        if (grabActive) {
            if (const Ball* gb = world.findBallById(grabbedBallId)) {
                drawThickSegment(window, mapper.toScreen(gb->pos), mouseScreen, 1.8f, sf::Color(255, 225, 110, 210));
            }
        }

        // Slingshot preview + predicted trajectory curve
        if (slingshotActive) {
            const Vec2 launchVel = (slingshotAnchorWorld - mouseWorld) * 3.6f;
            const auto previewPts = world.predictTrajectory(
                slingshotAnchorWorld, launchVel, spawnRadius, activeMaterial, 2.0f, 65);
            for (std::size_t i = 0; i < previewPts.size(); ++i) {
                const float fade = 1.0f - static_cast<float>(i) / static_cast<float>(previewPts.size());
                sf::CircleShape dot(2.4f, 10);
                dot.setOrigin({2.4f, 2.4f});
                dot.setPosition(mapper.toScreen(previewPts[i]));
                dot.setFillColor(sf::Color(255, 235, 135, static_cast<std::uint8_t>(220.0f * fade)));
                window.draw(dot);
            }

            const sf::Vector2f anchorSc = mapper.toScreen(slingshotAnchorWorld);
            const float rPx = spawnRadius * kPixelsPerMeter;
            sf::CircleShape ghost(rPx, 36);
            ghost.setOrigin({rPx, rPx});
            ghost.setPosition(anchorSc);
            ghost.setFillColor(toSfColor(kMaterials[activeMaterial].color, 110, true));
            ghost.setOutlineThickness(1.6f);
            ghost.setOutlineColor(sf::Color(255, 255, 255, 200));
            window.draw(ghost);

            drawArrow(window, mouseScreen, anchorSc, sf::Color(255, 215, 95));
        }

        // Impact sparks
        for (const auto& sp : sparks) {
            const float alphaF = std::clamp(1.0f - sp.life / sp.maxLife, 0.0f, 1.0f);
            sf::CircleShape p(2.2f * alphaF + 0.8f, 8);
            p.setOrigin({p.getRadius(), p.getRadius()});
            p.setPosition(mapper.toScreen(sp.pos));
            p.setFillColor(scaleColor(sp.color, 1.35f, static_cast<std::uint8_t>(245.0f * alphaF)));
            window.draw(p);
        }

        // ===================== LIVE STACKED ENERGY GRAPH (Top-Right) =====================
        {
            const float gw = 300.0f;
            const float gh = 118.0f;
            const float gx = mapper.winW - gw - 18.0f;
            const float gy = 14.0f;

            sf::RectangleShape panel({gw, gh + 32.0f});
            panel.setPosition({gx, gy});
            panel.setFillColor(sf::Color(14, 19, 28, 215));
            panel.setOutlineThickness(1.0f);
            panel.setOutlineColor(sf::Color(68, 84, 112, 190));
            window.draw(panel);

            float maxE = 1.0f;
            for (const auto& s : energyHistory) {
                maxE = std::max(maxE, s.total * 1.12f);
            }

            const float plotTop = gy + 24.0f;
            const float plotBot = plotTop + gh - 8.0f;
            const float plotH = plotBot - plotTop;
            const float plotLeft = gx + 10.0f;
            const float plotW = gw - 20.0f;

            if (energyHistory.size() >= 2) {
                sf::VertexArray peStrip(sf::PrimitiveType::TriangleStrip);
                sf::VertexArray keStrip(sf::PrimitiveType::TriangleStrip);
                sf::VertexArray rotStrip(sf::PrimitiveType::TriangleStrip);
                sf::VertexArray totalLine(sf::PrimitiveType::LineStrip);

                const std::size_t n = energyHistory.size();
                for (std::size_t i = 0; i < n; ++i) {
                    const float x = plotLeft + (static_cast<float>(i) / static_cast<float>(kMaxEnergySamples - 1)) * plotW;
                    const auto& e = energyHistory[i];

                    const float y0 = plotBot;
                    const float yPe = plotBot - std::clamp(e.potential / maxE, 0.0f, 1.0f) * plotH;
                    const float yKe = plotBot - std::clamp((e.potential + e.translationalKE) / maxE, 0.0f, 1.0f) * plotH;
                    const float yTot = plotBot - std::clamp(e.total / maxE, 0.0f, 1.0f) * plotH;

                    peStrip.append(sf::Vertex{{x, y0}, sf::Color(66, 145, 255, 130), {}});
                    peStrip.append(sf::Vertex{{x, yPe}, sf::Color(66, 145, 255, 165), {}});

                    keStrip.append(sf::Vertex{{x, yPe}, sf::Color(72, 232, 152, 130), {}});
                    keStrip.append(sf::Vertex{{x, yKe}, sf::Color(72, 232, 152, 165), {}});

                    rotStrip.append(sf::Vertex{{x, yKe}, sf::Color(255, 158, 68, 150), {}});
                    rotStrip.append(sf::Vertex{{x, yTot}, sf::Color(255, 158, 68, 185), {}});

                    totalLine.append(sf::Vertex{{x, yTot}, sf::Color(245, 250, 255, 235), {}});
                }

                window.draw(peStrip);
                window.draw(keStrip);
                window.draw(rotStrip);
                window.draw(totalLine);
            }

            if (hasFont) {
                const EnergySnapshot nowE = world.computeSystemEnergy();
                sf::Text title(font, "SYSTEM ENERGY  E=" + fmtFloat(nowE.total, 1) + " J", 12);
                title.setFillColor(sf::Color(230, 238, 250));
                title.setPosition({gx + 10.0f, gy + 5.0f});
                window.draw(title);

                sf::Text legend(font, "PE(blue)  KE_lin(green)  KE_rot(orange)", 10);
                legend.setFillColor(sf::Color(165, 180, 205));
                legend.setPosition({gx + 10.0f, plotBot + 3.0f});
                window.draw(legend);
            }
        }

        // ===================== TELEMETRY & CONTROLS HUD (Top-Left) =====================
        if (hasFont) {
            const float effE = world.config().useGlobalRestitution
                                   ? world.config().globalRestitution
                                   : kMaterials[activeMaterial].restitution;

            std::ostringstream hud;
            hud << "BOUNCY-BALL 3.0  [" << static_cast<int>(std::round(fpsSmooth)) << " FPS]"
                << (paused ? "  [PAUSED]" : "")
                << (audio.isMuted() ? "  [MUTED]" : "") << "\n"
                << "Balls: " << world.balls().size()
                << "  |  Ramps: " << world.ramps().size()
                << "  |  Gravity: " << kGravityPresets[gravityIdx].name << "\n"
                << "Spawn [1-6]: " << kMaterials[activeMaterial].name
                << "  (R=" << fmtFloat(spawnRadius, 2) << "m, spin=" << fmtFloat(spawnSpin, 1) << " rad/s)\n"
                << "Restitution e: " << fmtFloat(effE, 2)
                << (world.config().useGlobalRestitution ? " (global override)" : " (material)")
                << "  |  Air Drag+Magnus: " << (world.config().airDragEnabled ? "ON" : "OFF");

            if (selectedBall) {
                const float gMag = world.config().gravity.length();
                const float ke = selectedBall->translationalKE() + selectedBall->rotationalKE();
                const float pe = selectedBall->potentialEnergy(gMag);
                hud << "\n----------------------------------------------------\n"
                    << "SELECTED BALL #" << selectedBall->id
                    << " (" << kMaterials[selectedBall->materialIdx].name
                    << ", m=" << fmtFloat(selectedBall->mass, 2) << " kg)\n"
                    << "Pos: (" << fmtFloat(selectedBall->pos.x, 2) << ", "
                    << fmtFloat(selectedBall->heightAboveFloor(), 2) << ") m"
                    << "  |  Speed: " << fmtFloat(selectedBall->vel.length(), 2) << " m/s"
                    << "  |  Spin: " << fmtFloat(selectedBall->angularVel, 1) << " rad/s\n"
                    << "KE: " << fmtFloat(ke, 2) << " J  |  PE: " << fmtFloat(pe, 2)
                    << " J  |  Total: " << fmtFloat(ke + pe, 2) << " J\n"
                    << "Bounces: " << selectedBall->bounceCount
                    << "  |  Last Apex: " << fmtFloat(selectedBall->lastBounceApex, 3) << " m"
                    << "  |  dApex: " << fmtFloat(selectedBall->lastApexDiff, 3) << " m";
            }

            if (showHelp) {
                hud << "\n----------------------------------------------------\n"
                    << "LMB Drag Empty : Slingshot launch (Wheel=size, Q/E=spin)\n"
                    << "LMB Drag Ball  : Spring-grab & fling ball\n"
                    << "RMB Drag / Click: Draw bouncy ramp / Delete ball or ramp\n"
                    << "[1-6] Material  [Up/Down] e  [[/]] Gravity  [D] Drag  [P] Elastic e=1\n"
                    << "[Space] Pause  [S] Step  [B] Burst  [R] Reset 2.0  [V] Vectors  [M] Mute  [H] Hide Help";
            } else {
                hud << "\n[Press H for controls]";
            }

            const std::string hudStr = hud.str();
            const std::size_t lineCount = static_cast<std::size_t>(std::count(hudStr.begin(), hudStr.end(), '\n')) + 1;

            sf::RectangleShape card({520.0f, static_cast<float>(lineCount) * 16.2f + 16.0f});
            card.setPosition({kLeftMarginPx + 10.0f, 14.0f});
            card.setFillColor(sf::Color(14, 19, 28, 212));
            card.setOutlineThickness(1.0f);
            card.setOutlineColor(sf::Color(68, 84, 112, 185));
            window.draw(card);

            sf::Text txt(font, hudStr, 12);
            txt.setLineSpacing(1.12f);
            txt.setFillColor(sf::Color(224, 233, 246));
            txt.setPosition({kLeftMarginPx + 20.0f, 21.0f});
            window.draw(txt);
        }

        window.display();

        static int renderedFrames = 0;
        if (smokeTest && ++renderedFrames >= 5) {
            window.close();
        }
    }

    return 0;
}
