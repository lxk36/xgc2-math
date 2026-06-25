#ifndef XGC2_MATH_SLEW_RATE_LIMITER_HPP
#define XGC2_MATH_SLEW_RATE_LIMITER_HPP

#include <algorithm>
#include <cmath>

namespace xgc2_math {

class SlewRateLimiter {
  public:
    SlewRateLimiter() = default;

    explicit SlewRateLimiter(double max_rate_per_second, double initial_value = 0.0) {
        reset(max_rate_per_second, initial_value);
    }

    void reset(double max_rate_per_second, double initial_value = 0.0) {
        setMaxRatePerSecond(max_rate_per_second);
        resetState(initial_value);
    }

    void setMaxRatePerSecond(double max_rate_per_second) {
        max_rate_per_second_ = std::isfinite(max_rate_per_second) ? std::max(0.0, max_rate_per_second) : 0.0;
    }

    void resetState(double value = 0.0) {
        if (!std::isfinite(value)) {
            value_ = 0.0;
            initialized_ = false;
            return;
        }

        value_ = value;
        initialized_ = true;
    }

    double filter(double input, double dt_s) {
        if (!std::isfinite(input)) {
            return value_;
        }
        if (!initialized_) {
            resetState(input);
            return value_;
        }
        if (!std::isfinite(dt_s) || dt_s <= 0.0) {
            return value_;
        }
        if (max_rate_per_second_ <= 0.0) {
            return value_;
        }

        const double max_delta = max_rate_per_second_ * dt_s;
        if (!std::isfinite(max_delta)) {
            return value_;
        }

        value_ += std::clamp(input - value_, -max_delta, max_delta);
        return value_;
    }

    double value() const { return value_; }

    double maxRatePerSecond() const { return max_rate_per_second_; }

    bool initialized() const { return initialized_; }

  private:
    double max_rate_per_second_{0.0};
    double value_{0.0};
    bool initialized_{false};
};

} // namespace xgc2_math

#endif // XGC2_MATH_SLEW_RATE_LIMITER_HPP
