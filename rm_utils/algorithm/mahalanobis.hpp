#pragma once

#include <cmath>
#include <optional>

#include <Eigen/Cholesky>

namespace rm_utils::util {

template <class TVec, class TMat>
inline auto mahalanobis_distance(TVec const& innovation, TMat const& covariance)
    -> std::optional<double> {
    auto const ldlt = covariance.ldlt();
    auto solved     = ldlt.solve(innovation);
    auto distance   = innovation.dot(solved);
    if (!std::isfinite(distance)) return std::nullopt;
    return distance;
}

}
