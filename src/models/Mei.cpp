
#include "models.h"

MeiModel::MeiModel()
{
    xi = 0;
    p1 = 0;
    p2 = 0;
    k1 = 0;
    k2 = 0;
}

MeiModel::MeiModel(double xi, double p1, double p2, double k1, double k2, cv::Vec2d centerOffset, cv::Matx22d stretchMatrix)
{
    setIntrinsics(xi, p1, p2, k1, k2, centerOffset, stretchMatrix);
}

void MeiModel::setIntrinsics(double xi, double p1, double p2, double k1, double k2, cv::Vec2d centerOffset, cv::Matx22d stretchMatrix)
{
    this->xi = xi;
    this->p1 = p1;
    this->p2 = p2;
    this->k1 = k1;
    this->k2 = k2;
    this->stretchMatrix = stretchMatrix;
    this->centerOffset = centerOffset; 
}

cv::Point2d MeiModel::projectWorldToPixel(cv::Mat worldPoint)
{
	cv::normalize(worldPoint, worldPoint);
	worldPoint.at<float>(2) += xi;

	double x = worldPoint.at<float>(0) / worldPoint.at<float>(2);
	double y = worldPoint.at<float>(1) / worldPoint.at<float>(2);

    double _x, _y, _x2, _y2, _xy, _rho, rad_dist;
    _x2 = x * x;
    _y2 = y * y;
    _xy = x * y;
    _rho = _x2 + _y2;
    rad_dist = k1 * _rho + k2 * _rho * _rho;

    _x = x * rad_dist + 2 * p1 * _xy + p2 * (_rho + 2 * _x2);
    _y = y * rad_dist + 2 * p2 * _xy + p1 * (_rho + 2 * _y2);

    x += _x;
    y += _y;

    cv::Point fypixel(stretchMatrix * cv::Vec2d(x, -y) + centerOffset);        // technically could do toCorner's job, but I'll keep it simple for now
    //toCorner(fypixel, oldSize);

	return fypixel;
}

cv::Mat MeiModel::projectPixelToWorld(cv::Point pixel)
{
    return cv::Mat();
}
