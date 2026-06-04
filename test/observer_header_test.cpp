#include <cassert>
#include <cmath>

#include <xgc2_observer/observer.hpp>

namespace {

void testButterworth()
{
    xgc2_observer::SecondOrderButterworthLowPass filter(5.0, 0.0);
    double y = 0.0;
    for (int i = 0; i < 200; ++i) {
        y = filter.filter(1.0, 0.01);
        assert(std::isfinite(y));
    }
    assert(y > 0.95);

    const double held = filter.filter(std::numeric_limits<double>::quiet_NaN(), 0.01);
    assert(std::isfinite(held));
}

void testDifferentiator()
{
    xgc2_observer::DifferentiatorOptions options;
    options.min_dt_s = 0.001;
    options.max_dt_s = 0.1;
    options.max_input_step = 1.0;
    options.max_derivative = 20.0;
    options.derivative_cutoff_hz = 10.0;

    xgc2_observer::Differentiator differentiator(options);
    auto sample = differentiator.update(0.0, 0.02);
    assert(sample.status == xgc2_observer::SampleStatus::kInitialized);

    sample = differentiator.update(0.1, 0.02);
    assert(sample.status == xgc2_observer::SampleStatus::kAccepted);
    assert(sample.measurement_accepted);

    const double derivative = sample.derivative;
    sample = differentiator.update(10.0, 0.02);
    assert(sample.status == xgc2_observer::SampleStatus::kHeldOutlier);
    assert(!sample.measurement_accepted);
    assert(std::fabs(sample.derivative - derivative) < 1.0e-12);

    sample = differentiator.update(0.2, 0.0);
    assert(sample.status == xgc2_observer::SampleStatus::kHeldInvalidDt);
}

void testPositionVelocityObserver()
{
    xgc2_observer::PositionVelocityObserverOptions options;
    options.position_gain = 0.45;
    options.velocity_gain = 0.12;
    options.max_position_residual = 2.0;
    options.max_velocity = 5.0;

    xgc2_observer::PositionVelocityLuenbergerObserver observer(options);
    auto estimate = observer.update(0.0, 0.02);
    assert(estimate.status == xgc2_observer::SampleStatus::kInitialized);

    for (int i = 1; i <= 200; ++i) {
        estimate = observer.update(0.5 * i * 0.02, 0.02);
        assert(estimate.status == xgc2_observer::SampleStatus::kAccepted);
        assert(std::isfinite(estimate.position));
        assert(std::isfinite(estimate.velocity));
    }

    assert(observer.velocity() > 0.2);
    assert(observer.velocity() < 1.0);

    estimate = observer.update(100.0, 0.02);
    assert(estimate.status == xgc2_observer::SampleStatus::kHeldOutlier);
    assert(!estimate.measurement_accepted);
}

}  // namespace

int main()
{
    testButterworth();
    testDifferentiator();
    testPositionVelocityObserver();
    return 0;
}
