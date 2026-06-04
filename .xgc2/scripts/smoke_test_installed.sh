#!/usr/bin/env bash

set -euo pipefail

dpkg -s libxgc2-observer-dev >/dev/null

test -f /usr/include/xgc2_observer/observer.hpp
test -f /usr/lib/cmake/xgc2_observer/xgc2_observerConfig.cmake

probe_dir="${XGC2_OBSERVER_SMOKE_DIR:-$(mktemp -d -t xgc2-observer-smoke-XXXXXX)}"
mkdir -p "${probe_dir}"

cat > "${probe_dir}/CMakeLists.txt" <<'CMAKE'
cmake_minimum_required(VERSION 3.16)
project(xgc2_observer_probe LANGUAGES CXX)

find_package(xgc2_observer REQUIRED CONFIG)
xgc2_observer_require()

add_executable(link_probe link_probe.cpp)
target_compile_features(link_probe PRIVATE cxx_std_17)
target_link_libraries(link_probe PRIVATE xgc2_observer::observer)
CMAKE

cat > "${probe_dir}/link_probe.cpp" <<'CPP'
#include <xgc2_observer/observer.hpp>

int main()
{
  xgc2_observer::SecondOrderButterworthLowPass filter(5.0, 0.0);
  xgc2_observer::Differentiator diff;
  xgc2_observer::PositionVelocityLuenbergerObserver observer;

  const double filtered = filter.filter(1.0, 0.01);
  const auto derivative = diff.update(filtered, 0.01);
  const auto estimate = observer.update(filtered, 0.01);

  return derivative.measurement_accepted && estimate.measurement_accepted ? 0 : 1;
}
CPP

cmake -S "${probe_dir}" -B "${probe_dir}/build"
cmake --build "${probe_dir}/build" -- -j2
"${probe_dir}/build/link_probe"

echo "libxgc2-observer-dev installed smoke test passed."
