#pragma once

#include "xgc2_math/trajectory/analytic/detail.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>

namespace xgc2_math::trajectory {

struct LineCurveParameters3 {
    uint32_t flags{kFlagNone};
    double duration{8.0};
    Eigen::Vector3d start{Eigen::Vector3d::Zero()};
    Eigen::Vector3d start_velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d target{Eigen::Vector3d(1.0, 1.0, 1.0)};
    Eigen::Vector3d target_velocity{Eigen::Vector3d::Zero()};
};

class LineCurveEvaluator3 final : public TrajectoryEvaluator3 {
  public:
    explicit LineCurveEvaluator3(const LineCurveParameters3& params = {}) : params_(params) {
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = 8.0;
        }
        const double T = std::max(analytic_detail::kMinDuration, params_.duration);
        const Eigen::Vector3d a_delta = params_.target - params_.start - params_.start_velocity * T;
        const Eigen::Vector3d d_delta = params_.start_velocity - params_.target_velocity;
        a0_ = params_.start;
        a1_ = params_.start_velocity;
        a3_ = (10.0 * a_delta + 4.0 * d_delta * T) / (T * T * T);
        a4_ = -(15.0 * a_delta + 7.0 * d_delta * T) / (T * T * T * T);
        a5_ = (3.0 * (2.0 * a_delta + d_delta * T)) / (T * T * T * T * T);
    }

    bool evaluate(double t, FlatOutput3& output) const override {
        if (!analytic_detail::finiteScalar(t)) {
            output.flags |= kFlagInvalidInput;
            return false;
        }
        t = analytic_detail::clamp(t, 0.0, params_.duration);
        output = FlatOutput3{};
        const double t2 = t * t;
        const double t3 = t2 * t;
        const double t4 = t3 * t;
        const double t5 = t4 * t;
        output.position = a0_ + a1_ * t + a3_ * t3 + a4_ * t4 + a5_ * t5;
        output.velocity = a1_ + 3.0 * a3_ * t2 + 4.0 * a4_ * t3 + 5.0 * a5_ * t4;
        output.acceleration = 6.0 * a3_ * t + 12.0 * a4_ * t2 + 20.0 * a5_ * t3;
        output.jerk = 6.0 * a3_ + 24.0 * a4_ * t + 60.0 * a5_ * t2;
        output.snap = 24.0 * a4_ + 120.0 * a5_ * t;
        analytic_detail::fillYawFromVelocity(output);
        output.flags |= params_.flags;
        return TrajectoryValidator3::finite(output);
    }

    double duration() const override { return params_.duration; }
    TrajectoryModelType type() const override { return TrajectoryModelType::kAnalytic; }
    uint32_t flags() const override { return params_.flags; }
    const LineCurveParameters3& params() const { return params_; }

  private:
    LineCurveParameters3 params_;
    Eigen::Vector3d a0_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d a1_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d a3_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d a4_{Eigen::Vector3d::Zero()};
    Eigen::Vector3d a5_{Eigen::Vector3d::Zero()};
};

struct LemniscateCurveParameters3 {
    uint32_t flags{kFlagNone};
    double duration{20.0};
    Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
    double radius{1.0};
    double omega{0.9};
    double height{1.0};
};

class LemniscateCurveEvaluator3 final : public TrajectoryEvaluator3 {
  public:
    explicit LemniscateCurveEvaluator3(const LemniscateCurveParameters3& params = {}) : params_(params) {
        params_.radius = analytic_detail::safeRadius(params_.radius);
        params_.omega = std::abs(params_.omega);
        if (!analytic_detail::finiteScalar(params_.omega) || params_.omega <= 0.0) {
            params_.omega = 0.9;
        }
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = 20.0;
        }
    }

    bool evaluate(double t, FlatOutput3& output) const override {
        if (!analytic_detail::finiteScalar(t)) {
            output.flags |= kFlagInvalidInput;
            return false;
        }
        t = analytic_detail::clamp(t, 0.0, params_.duration);
        output = FlatOutput3{};
        const double a = params_.radius;
        const double w = params_.omega;
        const double wt = w * t;
        const double sin_wt = std::sin(wt);
        const double cos_wt = std::cos(wt);
        const double sin_2wt = std::sin(2.0 * wt);
        const double cos_2wt = std::cos(2.0 * wt);
        output.position = params_.origin + Eigen::Vector3d(a * sin_wt, a * sin_wt * cos_wt, params_.height);
        output.velocity = Eigen::Vector3d(a * w * cos_wt, a * w * cos_2wt, 0.0);
        output.acceleration = Eigen::Vector3d(-a * w * w * sin_wt, -2.0 * a * w * w * sin_2wt, 0.0);
        output.jerk = Eigen::Vector3d(-a * w * w * w * cos_wt, -4.0 * a * w * w * w * cos_2wt, 0.0);
        output.snap = Eigen::Vector3d(a * w * w * w * w * sin_wt, 8.0 * a * w * w * w * w * sin_2wt, 0.0);
        analytic_detail::fillYawFromVelocity(output);
        output.flags |= params_.flags;
        return TrajectoryValidator3::finite(output);
    }

    double duration() const override { return params_.duration; }
    TrajectoryModelType type() const override { return TrajectoryModelType::kAnalytic; }
    uint32_t flags() const override { return params_.flags; }
    const LemniscateCurveParameters3& params() const { return params_; }

  private:
    LemniscateCurveParameters3 params_;
};

struct HelixCurveParameters3 {
    uint32_t flags{kFlagNone};
    double duration{25.0};
    Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
    double radius{1.0};
    double omega{1.5};
    double linear_scale{10.0};
};

class HelixYzCurveEvaluator3 final : public TrajectoryEvaluator3 {
  public:
    explicit HelixYzCurveEvaluator3(const HelixCurveParameters3& params = {}) : params_(params) { sanitize(); }

    bool evaluate(double t, FlatOutput3& output) const override {
        if (!analytic_detail::finiteScalar(t)) {
            output.flags |= kFlagInvalidInput;
            return false;
        }
        t = analytic_detail::clamp(t, 0.0, params_.duration);
        output = FlatOutput3{};
        const double r = params_.radius;
        const double w = params_.omega;
        const double s = params_.linear_scale;
        const double wt = w * t;
        output.position = params_.origin + Eigen::Vector3d(t / s, r * std::cos(wt), r * std::sin(wt));
        output.velocity = Eigen::Vector3d(1.0 / s, -r * w * std::sin(wt), r * w * std::cos(wt));
        output.acceleration = Eigen::Vector3d(0.0, -r * w * w * std::cos(wt), -r * w * w * std::sin(wt));
        output.jerk = Eigen::Vector3d(0.0, r * w * w * w * std::sin(wt), -r * w * w * w * std::cos(wt));
        output.snap = Eigen::Vector3d(0.0, r * w * w * w * w * std::cos(wt), r * w * w * w * w * std::sin(wt));
        analytic_detail::fillYawFromVelocity(output);
        output.flags |= params_.flags;
        return TrajectoryValidator3::finite(output);
    }

    double duration() const override { return params_.duration; }
    TrajectoryModelType type() const override { return TrajectoryModelType::kAnalytic; }
    uint32_t flags() const override { return params_.flags; }
    const HelixCurveParameters3& params() const { return params_; }

  private:
    void sanitize() {
        params_.radius = analytic_detail::safeRadius(params_.radius);
        params_.omega = std::abs(params_.omega);
        params_.linear_scale = std::max(analytic_detail::kMinDuration, std::abs(params_.linear_scale));
        if (!analytic_detail::finiteScalar(params_.omega) || params_.omega <= 0.0) {
            params_.omega = 1.5;
        }
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = 25.0;
        }
    }

    HelixCurveParameters3 params_;
};

class HelixXyCurveEvaluator3 final : public TrajectoryEvaluator3 {
  public:
    explicit HelixXyCurveEvaluator3(const HelixCurveParameters3& params = {}) : params_(params) { sanitize(); }

    bool evaluate(double t, FlatOutput3& output) const override {
        if (!analytic_detail::finiteScalar(t)) {
            output.flags |= kFlagInvalidInput;
            return false;
        }
        t = analytic_detail::clamp(t, 0.0, params_.duration);
        output = FlatOutput3{};
        const double r = params_.radius;
        const double w = params_.omega;
        const double s = params_.linear_scale;
        const double wt = w * t;
        output.position = params_.origin + Eigen::Vector3d(r * std::cos(wt), r * std::sin(wt), t / s);
        output.velocity = Eigen::Vector3d(-r * w * std::sin(wt), r * w * std::cos(wt), 1.0 / s);
        output.acceleration = Eigen::Vector3d(-r * w * w * std::cos(wt), -r * w * w * std::sin(wt), 0.0);
        output.jerk = Eigen::Vector3d(r * w * w * w * std::sin(wt), -r * w * w * w * std::cos(wt), 0.0);
        output.snap = Eigen::Vector3d(r * w * w * w * w * std::cos(wt), r * w * w * w * w * std::sin(wt), 0.0);
        analytic_detail::fillYawFromVelocity(output);
        output.flags |= params_.flags;
        return TrajectoryValidator3::finite(output);
    }

    double duration() const override { return params_.duration; }
    TrajectoryModelType type() const override { return TrajectoryModelType::kAnalytic; }
    uint32_t flags() const override { return params_.flags; }
    const HelixCurveParameters3& params() const { return params_; }

  private:
    void sanitize() {
        params_.radius = analytic_detail::safeRadius(params_.radius);
        params_.omega = std::abs(params_.omega);
        params_.linear_scale = std::max(analytic_detail::kMinDuration, std::abs(params_.linear_scale));
        if (!analytic_detail::finiteScalar(params_.omega) || params_.omega <= 0.0) {
            params_.omega = 0.9;
        }
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = 25.0;
        }
    }

    HelixCurveParameters3 params_;
};

struct TorusKnotCurveParameters3 {
    uint32_t flags{kFlagNone};
    double duration{35.0};
    Eigen::Vector3d origin{Eigen::Vector3d::Zero()};
    double omega{0.9};
    double scale{0.3};
};

class TorusKnotCurveEvaluator3 final : public TrajectoryEvaluator3 {
  public:
    explicit TorusKnotCurveEvaluator3(const TorusKnotCurveParameters3& params = {}) : params_(params) {
        params_.omega = std::abs(params_.omega);
        params_.scale = std::abs(params_.scale);
        if (!analytic_detail::finiteScalar(params_.omega) || params_.omega <= 0.0) {
            params_.omega = 0.9;
        }
        if (!analytic_detail::finiteScalar(params_.scale) || params_.scale <= 0.0) {
            params_.scale = 0.3;
        }
        if (!analytic_detail::finiteScalar(params_.duration) || params_.duration <= 0.0) {
            params_.duration = 35.0;
        }
    }

    bool evaluate(double t, FlatOutput3& output) const override {
        if (!analytic_detail::finiteScalar(t)) {
            output.flags |= kFlagInvalidInput;
            return false;
        }
        t = analytic_detail::clamp(t, 0.0, params_.duration);
        output = FlatOutput3{};
        const double w = params_.omega;
        const double sc = params_.scale;
        const double wt = w * t;
        output.position =
            params_.origin + sc * Eigen::Vector3d(std::sin(wt) + 2.0 * std::sin(2.0 * wt),
                                                  std::cos(wt) - 2.0 * std::cos(2.0 * wt), 4.0 + std::sin(3.0 * wt));
        output.velocity =
            sc * Eigen::Vector3d(w * std::cos(wt) + 4.0 * w * std::cos(2.0 * wt),
                                 -w * std::sin(wt) + 4.0 * w * std::sin(2.0 * wt), 3.0 * w * std::cos(3.0 * wt));
        output.acceleration = sc * Eigen::Vector3d(-w * w * std::sin(wt) - 8.0 * w * w * std::sin(2.0 * wt),
                                                   -w * w * std::cos(wt) + 8.0 * w * w * std::cos(2.0 * wt),
                                                   -9.0 * w * w * std::sin(3.0 * wt));
        output.jerk = sc * Eigen::Vector3d(-w * w * w * std::cos(wt) - 16.0 * w * w * w * std::cos(2.0 * wt),
                                           w * w * w * std::sin(wt) - 16.0 * w * w * w * std::sin(2.0 * wt),
                                           -27.0 * w * w * w * std::cos(3.0 * wt));
        output.snap = sc * Eigen::Vector3d(w * w * w * w * std::sin(wt) + 32.0 * w * w * w * w * std::sin(2.0 * wt),
                                           w * w * w * w * std::cos(wt) - 32.0 * w * w * w * w * std::cos(2.0 * wt),
                                           81.0 * w * w * w * w * std::sin(3.0 * wt));
        analytic_detail::fillYawFromVelocity(output);
        output.flags |= params_.flags;
        return TrajectoryValidator3::finite(output);
    }

    double duration() const override { return params_.duration; }
    TrajectoryModelType type() const override { return TrajectoryModelType::kAnalytic; }
    uint32_t flags() const override { return params_.flags; }
    const TorusKnotCurveParameters3& params() const { return params_; }

  private:
    TorusKnotCurveParameters3 params_;
};

} // namespace xgc2_math::trajectory
