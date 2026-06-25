#ifndef XGC2_MATH_EXPONENTIAL_FILTER_HPP
#define XGC2_MATH_EXPONENTIAL_FILTER_HPP

#include <algorithm>
#include <cmath>

#include "xgc2_math/algebra/angle.hpp"

namespace xgc2_math {

class ExponentialLowPass {
  public:
    ExponentialLowPass() = default;

    explicit ExponentialLowPass(double cutoff_frequency_hz, double initial_value = 0.0) {
        reset(cutoff_frequency_hz, initial_value);
    }

    void reset(double cutoff_frequency_hz, double initial_value = 0.0) {
        setCutoffFrequencyHz(cutoff_frequency_hz);
        resetState(initial_value);
    }

    void setCutoffFrequencyHz(double cutoff_frequency_hz) {
        cutoff_frequency_hz_ = std::isfinite(cutoff_frequency_hz) ? std::max(0.0, cutoff_frequency_hz) : 0.0;
    }

    void resetState(double value = 0.0) {
        if (std::isfinite(value)) {
            value_ = value;
            initialized_ = true;
            return;
        }
        value_ = 0.0;
        initialized_ = false;
    }

    double filter(double input, double dt_s) {
        if (!std::isfinite(input)) {
            return value_;
        }

        if (!initialized_) {
            resetState(input);
            return value_;
        }

        // A non-positive cutoff disables filtering and follows the latest valid input.
        if (cutoff_frequency_hz_ <= 0.0) {
            resetState(input);
            return value_;
        }

        if (dt_s <= 0.0 || !std::isfinite(dt_s)) {
            return value_;
        }

        const double alpha = -std::expm1(-kTwoPi * cutoff_frequency_hz_ * dt_s);
        value_ += alpha * (input - value_);
        return value_;
    }

    double value() const { return value_; }

    double cutoffFrequencyHz() const { return cutoff_frequency_hz_; }

    bool initialized() const { return initialized_; }

  private:
    double cutoff_frequency_hz_{0.0};
    double value_{0.0};
    bool initialized_{false};
};

} // namespace xgc2_math

#endif // XGC2_MATH_EXPONENTIAL_FILTER_HPP
