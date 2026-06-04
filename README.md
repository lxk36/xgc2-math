# XGC2 Observer

`libxgc2-observer-dev` provides small C++17 numerical observer utilities for
robotics runtime code.  The package is ROS-independent so ROS1, ROS2, Gazebo,
PX4/MAVROS integration code, controllers, MATLAB bindings, and non-ROS CMake
projects can use the same installed headers.

The initial scope is deliberately narrow:

- low-pass filtering for weak timing jitter and noisy scalar signals;
- numerical differentiation that rejects invalid samples, bad `dt`, and large
  jumps before updating state;
- a position/velocity Luenberger-style observer for motion-capture and target
  tracking signals.

## Install

```bash
sudo apt update
sudo apt install libxgc2-observer-dev
```

The package installs:

```text
/usr/include/xgc2_observer/
/usr/lib/cmake/xgc2_observer/
```

## CMake Usage

```cmake
find_package(xgc2_observer REQUIRED CONFIG)

target_link_libraries(your_target
  PRIVATE
    xgc2_observer::observer
)
```

Include the umbrella header:

```cpp
#include <xgc2_observer/observer.hpp>
```

or individual headers:

```cpp
#include <xgc2_observer/butterworth_filter.hpp>
#include <xgc2_observer/differentiator.hpp>
#include <xgc2_observer/luenberger_observer.hpp>
```

## Algorithms

### `SecondOrderButterworthLowPass`

Second-order Butterworth low-pass filter with coefficients recomputed from the
current `dt`.  This makes it usable with small period jitter while preserving a
simple scalar API.

```cpp
xgc2_observer::SecondOrderButterworthLowPass filter(5.0, 0.0);
const double filtered = filter.filter(raw_value, dt_s);
```

Invalid input holds the last output.  Invalid `dt` or non-positive cutoff resets
the filter state to the current input because filtering semantics are not
defined in that case.

### `Differentiator`

Derivative estimator that updates only when the input and sampling interval pass
sanity checks.

```cpp
xgc2_observer::DifferentiatorOptions options;
options.max_input_step = 0.5;
options.max_derivative = 5.0;
options.derivative_cutoff_hz = 8.0;

xgc2_observer::Differentiator diff(options);
const auto sample = diff.update(position, dt_s);
```

The returned sample reports whether the measurement was accepted or held because
of invalid input, invalid `dt`, or an outlier.  State is updated only for
accepted samples.

### `PositionVelocityLuenbergerObserver`

One-dimensional position/velocity observer for target tracking and motion-capture
signals.  It predicts with optional acceleration input, then corrects position
and velocity from measured position residual.

```cpp
xgc2_observer::PositionVelocityObserverOptions options;
options.position_gain = 0.35;
options.velocity_gain = 0.08;
options.max_position_residual = 1.0;

xgc2_observer::PositionVelocityLuenbergerObserver observer(options);
const auto estimate = observer.update(measured_position, dt_s);
```

This is intended for scalar axes.  Multi-axis users should instantiate one
observer per axis or wrap it in a domain-specific vector type.

## Design Boundary

This package owns numerical signal conditioning and observer primitives.  It
does not own ROS topics, frame transforms, MAVROS routing, Gazebo model state,
or controller policy.  Callers are expected to set limits according to their
sensor and vehicle envelope.

## Build Locally

```bash
cmake -S . -B build
cmake --build build -- -j"$(nproc)"
(cd build && ctest --output-on-failure)
```

Package scripts:

```bash
./.xgc2/scripts/check_package_compliance.sh
./.xgc2/scripts/build_deb.sh
sudo apt-get install -y ./.ci/debs/libxgc2-observer-dev_*.deb
./.xgc2/scripts/smoke_test_installed.sh
```
