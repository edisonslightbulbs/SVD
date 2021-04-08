#include "svd.h"

std::pair<Eigen::JacobiSVD<Eigen::MatrixXd>, Eigen::MatrixXd> svd::compute(
    std::vector<Point>& points)
{
    /** compute centroid */
    Point centroid = Point::centroid(points);

    /** represent points as matrix */
    Eigen::MatrixXd translatedPointsMat(points.size(), R);
    int row = 0;
    for (auto point : points) {
        translatedPointsMat(row, xCol) = point.m_x;
        translatedPointsMat(row, yCol) = point.m_y;
        translatedPointsMat(row, zCol) = point.m_z;
        row++;
    }
    /**   translate points to direction vectors */
    translatedPointsMat.col(xCol).array() -= centroid.m_x;
    translatedPointsMat.col(yCol).array() -= centroid.m_y;
    translatedPointsMat.col(zCol).array() -= centroid.m_z;

    /** compute svd */
    int setting = Eigen::ComputeThinU | Eigen::ComputeThinV;
    Eigen::JacobiSVD<Eigen::MatrixXd> svd
        = translatedPointsMat.jacobiSvd(setting);

    /** return computed svd and translated points matrix */
    return { svd, translatedPointsMat };
}
