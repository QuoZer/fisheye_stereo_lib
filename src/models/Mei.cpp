
#include "models.h"

cv::Point2d MeiModel::projectWorldToPixel(cv::Mat worldPoint)
{
	cv::normalize(worldPoint, worldPoint);
	worldPoint.at<float>(2) += eps;

	double x = worldPoint.at<float>(0) / worldPoint.at<float>(2);
	double y = worldPoint.at<float>(1) / worldPoint.at<float>(2);

    double _x, _y, _x2, _y2, _xy, _rho, rad_dist;
    _x2 = x * x;
    _y2 = y * y;
    _xy = x * y;
    _rho = _x2 + _y2;
    rad_dist = mei_polynom[0] * _rho + mei_polynom[1] * _rho * _rho;

    _x = x * rad_dist + 2 * mei_polynom[2] * _xy + mei_polynom[3] * (_rho + 2 * _x2);
    _y = y * rad_dist + 2 * mei_polynom[3] * _xy + mei_polynom[2] * (_rho + 2 * _y2);

    x += _x;
    y += _y;

	return cv::Point2d(x, y);
}

