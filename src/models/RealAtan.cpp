
#include "models.h"

cv::Point2d RealAtanModel::projectWorldToPixel(cv::Mat worldPoint)
{
    double X = worldPoint.at<float>(0);
    double Y = worldPoint.at<float>(1);
    double Z = worldPoint.at<float>(2);
    //  sqrt destroys signs so I remember them here
    int8_t xSign = 1, ySign = 1;
    if (X == 0) X += 0.0001;
    else if (X < 0) xSign = -1;
    if (Y == 0) Y += 0.0001;
    else if (Y < 0) ySign = -1;
    double phi = atan2(sqrt(X * X + Y * Y), Z);
    double fisheye_fov = M_PI * (185 / 180);

    double xFocus = 1 * oldSize.width / fisheye_fov;
    double yFocus = 1 * oldSize.height / fisheye_fov;
    double u = xSign * xFocus * (phi) / sqrt((Y * Y) / (X * X) + 1);
    double v = ySign * yFocus * (phi) / sqrt((X * X) / (Y * Y) + 1);

    cv::Point fypixel(stretchMatrix * cv::Vec2d(u,  -v) + centerOffset);        // technically could do toCorner's job, but I'll keep it simple for now
    //toCorner(fypixel, oldSize);
    return fypixel;
}

void RealAtanModel::setIntrinsics(cv::Vec2d centerOffset, cv::Matx22d stretchMatrix)
{
    this->centerOffset = centerOffset;
    this->stretchMatrix = stretchMatrix;
}