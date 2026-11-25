#pragma once

#include <SFML/Audio/Sound.hpp>
#include <SFML/Audio/SoundBuffer.hpp>
#include <SFML/Audio/SoundChannel.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>

namespace bouncy {

// Procedural impact synthesizer: generates crisp rubbery/woody bounce waveforms in RAM
// with stereo panning, radius-scaled pitch, and impulse-scaled volume. Zero external assets!
class AudioSynth {
public:
    AudioSynth() {
        constexpr unsigned int kSampleRate = 44100;
        constexpr float kDuration = 0.095f;
        const std::size_t sampleCount = static_cast<std::size_t>(kSampleRate * kDuration);
        std::vector<std::int16_t> samples(sampleCount);

        constexpr float kTwoPi = 2.0f * std::numbers::pi_v<float>;
        for (std::size_t i = 0; i < sampleCount; ++i) {
            const float t = static_cast<float>(i) / static_cast<float>(kSampleRate);
            // Downward pitch sweep (membrane/rubber thud) + crisp transient click
            const float freq = 135.0f + 290.0f * std::exp(-t * 58.0f);
            const float envBody = std::exp(-t * 38.0f);
            const float envClick = std::exp(-t * 240.0f);

            const float body = std::sin(kTwoPi * freq * t);
            const float overtone = 0.35f * std::sin(kTwoPi * (freq * 2.4f) * t);
            const float transient = 0.45f * std::sin(kTwoPi * 980.0f * t) * envClick;

            const float wave = std::clamp((body + overtone) * envBody + transient, -1.0f, 1.0f);
            samples[i] = static_cast<std::int16_t>(wave * 26000.0f);
        }

        ready_ = buffer_.loadFromSamples(
            samples.data(),
            samples.size(),
            1,
            kSampleRate,
            {sf::SoundChannel::Mono});

        if (ready_) {
            voices_.reserve(kMaxVoices);
            for (std::size_t i = 0; i < kMaxVoices; ++i) {
                voices_.emplace_back(buffer_);
                voices_.back().setSpatializationEnabled(false);
            }
        }
    }

    void setMuted(bool muted) noexcept { muted_ = muted; }
    [[nodiscard]] bool isMuted() const noexcept { return muted_; }
    void toggleMute() noexcept { muted_ = !muted_; }

    void playImpact(float normalSpeed, float radiusMeters, float normalizedX) {
        if (!ready_ || muted_ || voices_.empty() || normalSpeed < 0.40f) {
            return;
        }

        sf::Sound& voice = voices_[cursor_];
        cursor_ = (cursor_ + 1) % voices_.size();

        const float volume = std::clamp((normalSpeed - 0.35f) * 9.5f, 3.0f, 85.0f);
        const float pitch = std::clamp(0.24f / std::max(0.06f, radiusMeters), 0.55f, 2.35f);
        const float pan = std::clamp(normalizedX * 2.0f - 1.0f, -0.85f, 0.85f);

        voice.stop();
        voice.setVolume(volume);
        voice.setPitch(pitch);
        voice.setPan(pan);
        voice.play();
    }

private:
    static constexpr std::size_t kMaxVoices = 12;

    sf::SoundBuffer buffer_{};
    std::vector<sf::Sound> voices_{};
    std::size_t cursor_{0};
    bool ready_{false};
    bool muted_{false};
};

} // namespace bouncy
