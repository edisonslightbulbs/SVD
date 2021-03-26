#include "svd.h"

std::pair<Eigen::JacobiSVD<Eigen::MatrixXd>, Eigen::MatrixXd> svd::compute(
    std::vector<Point>& points)
{
    /** compute centroid */
    Point centroid = Point::centroid(points);

    /** represent points as matrix */
    Eigen::MatrixXd pointsMat(points.size(), R);
    int row = 0;
    for (auto point : points) {
        pointsMat(row, xCol) = point.m_x;
        pointsMat(row, yCol) = point.m_y;
        pointsMat(row, zCol) = point.m_z;
        row++;
    }
    /**   translate points to direction vectors */
    pointsMat.col(xCol).array() -= centroid.m_x;
    pointsMat.col(yCol).array() -= centroid.m_y;
    pointsMat.col(zCol).array() -= centroid.m_z;

    /** compute svd */
    Eigen::JacobiSVD<Eigen::MatrixXd> svd
        = pointsMat.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV);

    /** return computed svd and computed on matrix */
    return { svd, pointsMat };
}
