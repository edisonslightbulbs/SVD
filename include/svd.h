#ifndef SVD_H
#define SVD_H

#include <vector>
#include <utility>
#include <Eigen/Dense>

#include "point.h"

namespace svd {
    std::pair<Eigen::JacobiSVD<Eigen::MatrixXd>, Eigen::MatrixXd> compute(std::vector<Point>& points);
};
#endif /* SVD_H */
