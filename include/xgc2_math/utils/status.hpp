#ifndef XGC2_MATH_STATUS_HPP
#define XGC2_MATH_STATUS_HPP

#include <algorithm>

namespace xgc2_math {

template <typename Status> struct StatusDescriptor {
    Status status;
    const char* name;
};

template <typename Status> struct StatusRegistry;

template <typename Status> inline const auto& registeredStatuses() {
    return StatusRegistry<Status>::statuses;
}

template <typename Status> inline const StatusDescriptor<Status>* statusDescriptor(Status status) {
    const auto& statuses = registeredStatuses<Status>();
    const auto it = std::find_if(statuses.begin(), statuses.end(), [status](const auto& descriptor) {
        return descriptor.status == status;
    });
    return it != statuses.end() ? &(*it) : nullptr;
}

template <typename Status> inline const char* toString(Status status) {
    const auto* descriptor = statusDescriptor(status);
    return descriptor != nullptr ? descriptor->name : "unknown";
}

} // namespace xgc2_math

#endif // XGC2_MATH_STATUS_HPP
