#include "svd.h"

std::pair<Eigen::JacobiSVD<Eigen::MatrixXf>, Eigen::MatrixXf> svd::compute(
    std::vector<Point>& points)
{
    /** compute centroid */
    Point centroid = Point::centroid(points);

    /** represent points as matrix */
    Eigen::MatrixXf vectors(points.size(), R);
    int row = 0;
    for (auto point : points) {
        vectors(row, xCol) = point.m_x;
        vectors(row, yCol) = point.m_y;
        vectors(row, zCol) = point.m_z;
        row++;
    }
    /**   translate points to direction vectors */
    vectors.col(xCol).array() -= centroid.m_x;
    vectors.col(yCol).array() -= centroid.m_y;
    vectors.col(zCol).array() -= centroid.m_z;

    /** compute svd */
    int setting = Eigen::ComputeThinU | Eigen::ComputeThinV;
    Eigen::JacobiSVD<Eigen::MatrixXf> svd = vectors.jacobiSvd(setting);

    /** return computed svd and translated points matrix */
    return { svd, vectors };
}
