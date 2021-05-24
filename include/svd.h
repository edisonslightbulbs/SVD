#ifndef SVD_H
#define SVD_H

#include <Eigen/Dense>
#include <utility>
#include <vector>

#include "point.h"

class svd {

private:
    std::vector<Point> m_points;
    const int C0 = 0; // single column
    const int C1 = 1; // single column
    const int C2 = 2; // single column
public:
    Point m_centroid;
    Eigen::MatrixXf m_vectors;
    Eigen::JacobiSVD<Eigen::MatrixXf> m_usv;

    explicit svd(const std::vector<Point>& points);

    Eigen::Vector3d getV3Normal();

    std::vector<Eigen::Vector3d> getUNormals();
};
#endif /* SVD_H */
