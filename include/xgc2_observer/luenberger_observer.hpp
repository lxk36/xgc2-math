#ifndef XGC2_OBSERVER_LUENBERGER_OBSERVER_HPP
#define XGC2_OBSERVER_LUENBERGER_OBSERVER_HPP

#include <cmath>
#include <limits>

#include "xgc2_observer/differentiator.hpp"

namespace xgc2_observer {

struct PositionVelocityObserverOptions {
    double position_gain{0.35};
    double velocity_gain{0.08};
    double min_dt_s{1.0e-4};
    double max_dt_s{0.5};
    double max_position_residual{std::numeric_limits<double>::infinity()};
    double max_velocity{std::numeric_limits<double>::infinity()};
};

struct PositionVelocityEstimate {
    double position{0.0};
    double velocity{0.0};
    double residual{0.0};
    SampleStatus status{SampleStatus::kInitialized};
    bool measurement_accepted{false};
};

class PositionVelocityLuenbergerObserver {
public:
    PositionVelocityLuenbergerObserver() = default;

    explicit PositionVelocityLuenbergerObserver(PositionVelocityObserverOptions options)
        : options_(options)
    {
    }

    void setOptions(PositionVelocityObserverOptions options)
    {
        options_ = options;
    }

    const PositionVelocityObserverOptions& options() const
    {
        return options_;
    }

    void reset()
    {
        initialized_ = false;
        position_ = 0.0;
        velocity_ = 0.0;
    }

    void reset(double position, double velocity = 0.0)
    {
        initialized_ = true;
        position_ = position;
        velocity_ = velocity;
    }

    PositionVelocityEstimate update(double measured_position, double dt_s, double acceleration = 0.0)
    {
        if (!std::isfinite(measured_position) || !std::isfinite(acceleration)) {
            return estimate(0.0, SampleStatus::kHeldInvalidInput, false);
        }

        if (!initialized_) {
            reset(measured_position, 0.0);
            return estimate(0.0, SampleStatus::kInitialized, true);
        }

        if (!validDt(dt_s)) {
            return estimate(0.0, SampleStatus::kHeldInvalidDt, false);
        }

        const double predicted_position = position_ + velocity_ * dt_s + 0.5 * acceleration * dt_s * dt_s;
        const double predicted_velocity = velocity_ + acceleration * dt_s;
        const double residual = measured_position - predicted_position;

        position_ = predicted_position;
        velocity_ = predicted_velocity;

        if (std::fabs(residual) > options_.max_position_residual) {
            clampVelocity();
            return estimate(residual, SampleStatus::kHeldOutlier, false);
        }

        position_ += options_.position_gain * residual;
        velocity_ += options_.velocity_gain * residual / dt_s;
        clampVelocity();
        return estimate(residual, SampleStatus::kAccepted, true);
    }

    double position() const
    {
        return position_;
    }

    double velocity() const
    {
        return velocity_;
    }

    bool initialized() const
    {
        return initialized_;
    }

private:
    bool validDt(double dt_s) const
    {
        return std::isfinite(dt_s) && dt_s >= options_.min_dt_s && dt_s <= options_.max_dt_s;
    }

    void clampVelocity()
    {
        if (std::fabs(velocity_) > options_.max_velocity) {
            velocity_ = std::copysign(options_.max_velocity, velocity_);
        }
    }

    PositionVelocityEstimate estimate(double residual, SampleStatus status, bool accepted) const
    {
        PositionVelocityEstimate output;
        output.position = position_;
        output.velocity = velocity_;
        output.residual = residual;
        output.status = status;
        output.measurement_accepted = accepted;
        return output;
    }

    PositionVelocityObserverOptions options_{};
    double position_{0.0};
    double velocity_{0.0};
    bool initialized_{false};
};

}  // namespace xgc2_observer

#endif  // XGC2_OBSERVER_LUENBERGER_OBSERVER_HPP
