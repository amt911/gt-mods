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
    read_setting(get, warn, "ahead_m", 8.0f, 100.0f, &s.ahead_m);
    // Below one ring interval the "pose that old" would not exist; past the ring's span it could never be found.
    read_setting(get, warn, "rewind_ms", kPoseEveryNSteps * kStepMs, (kRingCapacity - 2) * kPoseEveryNSteps * kStepMs, &s.rewind_ms);
#if defined(RESET_SABOTAGE)
    s.sabotage = RESET_SABOTAGE;  // the in-game check's sabotage builds (it must fail with them)
#endif
    return s;
}

bool is_good(const CarSample &s) {
    return std::isfinite(s.up_z) && std::isfinite(s.speed) && std::isfinite(s.yaw_rate) && std::isfinite(s.heading_cos) &&
           s.wheels_on_asphalt == 4 && s.up_z > kMinUpZ && s.speed > kMinSpeed && !s.flag_bit2 && std::fabs(s.yaw_rate) <= kMaxYawRate &&
           s.heading_cos > kMinHeadingCos;
}

bool ahead_on_asphalt(const RoadQuery &query, float x, float y, float z, float yaw, float ahead_m) {
    if (!std::isfinite(x + y + z + yaw + ahead_m) || ahead_m <= 0.0f) return false;
    const float fx = -std::sin(yaw), fy = std::cos(yaw), rx = std::cos(yaw), ry = std::sin(yaw);
    static const float kSides[] = {0.0f, -kSideMeters, kSideMeters};
    for (int i = 1; i <= static_cast<int>(std::ceil(ahead_m / 4.0f)); ++i) {
        const float d = std::min(4.0f * i, ahead_m);  // 4, 8 .. and the last step to ahead_m
        for (float side : kSides) {
            RoadPoint p;
            if (!query(x + fx * d + rx * side, y + fy * d + ry * side, &p) || !p.asphalt || !std::isfinite(p.ground) ||
                std::fabs(p.ground - (z - kGroundBelowOrigin)) >= kLevelTolerance)
                return false;
        }
    }
    return true;
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

const Pose *PoseRing::pick(std::int64_t now, int min_age, int max_age) const {
    const Pose *oldest = nullptr;
    for (const Item &it : items_)
        if (now - it.step <= max_age) {
            oldest = &it.pose;
            break;
        }
    if (oldest == nullptr) return nullptr;
    for (std::size_t i = items_.size(); i-- > 0;)
        if (now - items_[i].step >= min_age && now - items_[i].step <= max_age) return &items_[i].pose;
    return oldest;
}

Resetter::Resetter(const Settings &s)
    : s_(s), rewind_steps_(s.sabotage == 3 ? 0 : steps_for_ms(s.rewind_ms)), cooldown_steps_(steps_for_ms(s.cooldown_ms)), hold_(steps_for_ms(s.hold_ms)),
      ring_(kRingCapacity) {}

void Resetter::lose_car() {
    clear_ring();
    now_ = 0;
    last_reset_ = -1;
    have_prev_ = false;
    hold_ = HoldDetector(steps_for_ms(s_.hold_ms));
}

std::optional<Pose> Resetter::step(const CarSample &sample, const Pose &current, bool button_held, std::uint32_t hint0, std::uint32_t hint1) {
    ++now_;
    if (have_prev_ && std::hypot(std::hypot(current.v[0] - prev_pos_[0], current.v[1] - prev_pos_[1]), current.v[2] - prev_pos_[2]) > kJumpMeters)
        clear_ring();  // not our own restore (that updates prev_pos_): a respawn, a new attempt, a long gap
    prev_pos_[0] = current.v[0]; prev_pos_[1] = current.v[1]; prev_pos_[2] = current.v[2];
    have_prev_ = true;
    const bool press = hold_.update(button_held);
    const bool sabotage_newest = s_.sabotage == 2;
    if (now_ % kPoseEveryNSteps == 0 && (sabotage_newest || is_good(sample))) {
        if (deferred_ && !sabotage_newest) pending_.push_back(Candidate{now_, epoch_, current, hint0, hint1});  // the tick decides (look-ahead rule)
        else ring_.push(now_, current);
        if (pending_.size() > 8) pending_.erase(pending_.begin());
    }
    if (!press) return std::nullopt;
    if (last_reset_ >= 0 && now_ - last_reset_ < cooldown_steps_) return std::nullopt;
    const Pose *pick = ring_.pick(now_, sabotage_newest ? 0 : rewind_steps_, sabotage_newest ? (1 << 30) : kMaxPoseAgeSteps);
    if (pick == nullptr) return std::nullopt;
    const Pose restored = *pick;
    clear_ring();
    ring_.push(now_, restored);  // a second press right after goes back to the same place
    last_reset_ = now_;
    prev_pos_[0] = restored.v[0]; prev_pos_[1] = restored.v[1]; prev_pos_[2] = restored.v[2];
    return restored;
}

std::vector<Resetter::Candidate> Resetter::take_pending() {
    std::vector<Candidate> out;
    out.swap(pending_);
    return out;
}

void Resetter::accept(const Candidate &c) {
    if (c.epoch == epoch_) ring_.push(c.step, c.pose);
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

namespace reset {
bool SelectGate::visible(bool sel, bool racing, std::int64_t now) {
    if (!racing) {  // a menu, the pause screen, no player car: the game gets the pad as it is
        held_ = long_ = false;
        tap_left_ = 0;
        return sel;
    }
    if (sel) {
        if (!held_) {
            held_ = true;
            long_ = false;
            press_ = now;
            tap_left_ = 0;
        }
        if (now - press_ >= hold_steps_) long_ = true;
        return false;
    }
    if (held_) {
        held_ = false;
        if (!long_) tap_left_ = kTapReads;
        long_ = false;
    }
    if (tap_left_ > 0) {
        --tap_left_;
        return true;
    }
    return false;
}

}  // namespace reset
