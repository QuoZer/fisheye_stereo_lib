#include "models.hpp"

// equidistant fisheye 
cv::Point2d AtanModel::projectWorldToPixel(cv::Mat worldPoint)
{
    float wx = worldPoint.at<float>(0);
    float wy = worldPoint.at<float>(1);
    float wz = worldPoint.at<float>(2);
    //  sqrt destroys signs so I remember them here
    int8_t xSign = 1, ySign = 1;
    if (wx == 0) wx += 0.0001;
    else if (wx < 0) xSign = -1;
    if (wy == 0) wy += 0.0001;
    else if (wy < 0) ySign = -1;
    if (wz == 0) wz += 0.0001;
    //  fisheye focus
	double fisheye_fov = M_PI * (185/180);
    double xFocus = oldSize.width / fisheye_fov;       // TODO: check the logic begind this size
    double yFocus = oldSize.height / fisheye_fov;
	
    //  calculate the point location on fisheye image in central coordinates
    double wu = xSign * xFocus * atan(sqrt(wx * wx + wy * wy) / wz)
        / sqrt((wy * wy) / (wx * wx) + 1);
    double wv = ySign * yFocus * atan(sqrt(wx * wx + wy * wy) / wz)
        / sqrt((wx * wx) / (wy * wy) + 1);

    cv::Point fypixel(stretchMatrix * cv::Vec2d(wu, -wv) + centerOffset);	

    //  convert to corner coordinates
    //toCorner(fypixel, oldSize);

    return fypixel;
}



void AtanModel::setIntrinsics( cv::Vec2d centerOffset, cv::Matx22d stretchMatrix)
{
    this->centerOffset = centerOffset;
    this->stretchMatrix = stretchMatrix;
}
