#include "reset_logic.hpp"

#include <cmath>
#include <cstdlib>

namespace reset {
namespace {
bool parse_float(const char *text, float *out) {
    if (text == nullptr || *text == '\0') return false;
    char *end = nullptr;
    const float v = std::strtof(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(v)) return false;
    *out = v;
    return true;
}

void read_setting(const std::function<const char *(const char *)> &get, const std::function<void(const std::string &)> &warn,
                  const char *key, float lo, float hi, float *value) {
    const char *text = get(key);
    if (text == nullptr) return;
    float v;
    if (!parse_float(text, &v) || v < lo || v > hi) {
        warn(std::string("invalid ") + key + " = \"" + text + "\", keeping the default");
        return;
    }
    *value = v;
}
}  // namespace

int steps_for_ms(float ms) {
    if (!(ms > 0.0f)) return 0;
    return static_cast<int>(std::floor(ms / kStepMs + 1e-3f));
}

Settings parse_settings(const std::function<const char *(const char *)> &get,
                        const std::function<void(const std::string &)> &warn) {
    Settings s;
    read_setting(get, warn, "hold_ms", 0.0f, 5000.0f, &s.hold_ms);
    read_setting(get, warn, "cooldown_ms", 0.0f, 30000.0f, &s.cooldown_ms);
    // Below one ring interval the "pose that old" would not exist; past the ring's span it could never be found.
    read_setting(get, warn, "rewind_ms", kPoseEveryNSteps * kStepMs, (kRingCapacity - 2) * kPoseEveryNSteps * kStepMs, &s.rewind_ms);
#if defined(RESET_SABOTAGE)
    s.sabotage = RESET_SABOTAGE;  // the in-game check's sabotage builds (it must fail with them)
#endif
    return s;
}

bool is_good(const CarSample &s) {
    return std::isfinite(s.up_z) && std::isfinite(s.speed) && s.wheels_on_asphalt == 4 && s.up_z > kMinUpZ && s.speed > kMinSpeed &&
           !s.flag_bit2;
}

bool HoldDetector::update(bool held) {
    if (!held) {
        run_ = 0;
        fired_ = false;
        return false;
    }
    if (fired_) return false;
    if (++run_ < need_) return false;
    fired_ = true;
    return true;
}

void PoseRing::push(std::int64_t step, const Pose &pose) {
    items_.push_back(Item{step, pose});
    if (items_.size() > capacity_) items_.erase(items_.begin());
}

const Pose *PoseRing::pick(std::int64_t now, int min_age) const {
    if (items_.empty()) return nullptr;
    for (std::size_t i = items_.size(); i-- > 0;)
        if (now - items_[i].step >= min_age) return &items_[i].pose;
    return &items_.front().pose;
}

Resetter::Resetter(const Settings &s)
    : s_(s), rewind_steps_(steps_for_ms(s.rewind_ms)), cooldown_steps_(steps_for_ms(s.cooldown_ms)), hold_(steps_for_ms(s.hold_ms)),
      ring_(kRingCapacity) {}

void Resetter::lose_car() {
    ring_.clear();
    now_ = 0;
    last_reset_ = -1;
    hold_ = HoldDetector(steps_for_ms(s_.hold_ms));
}

std::optional<Pose> Resetter::step(const CarSample &sample, const Pose &current, bool button_held) {
    ++now_;
    const bool press = hold_.update(button_held);
    const bool sabotage_newest = s_.sabotage == 2;
    if (now_ % kPoseEveryNSteps == 0 && (sabotage_newest || is_good(sample))) ring_.push(now_, current);
    if (!press) return std::nullopt;
    if (last_reset_ >= 0 && now_ - last_reset_ < cooldown_steps_) return std::nullopt;
    const Pose *pick = ring_.pick(now_, sabotage_newest ? 0 : rewind_steps_);
    if (pick == nullptr) return std::nullopt;
    const Pose restored = *pick;
    ring_.clear();
    ring_.push(now_, restored);  // a second press right after goes back to the same place
    last_reset_ = now_;
    return restored;
}

std::optional<std::uint32_t> parse_fake(const char *env) {
    if (env == nullptr || *env == '\0') return std::nullopt;
    char *end = nullptr;
    const long v = std::strtol(env, &end, 10);
    if (end == env || *end != '\0' || v < 0) return std::nullopt;
    return static_cast<std::uint32_t>(v);
}

bool fake_held(std::uint32_t step, std::uint32_t from) { return step >= from && step < from + kFakeHeldSteps; }

}  // namespace reset
