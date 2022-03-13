#include "models.h"

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
    double xFocus = newSize.width / M_PI;
    double yFocus = newSize.height / M_PI;
    cv::Point projectionPoint(newSize.width / 2, newSize.height / 2);       //  initial value set to the image corner

    //  calculate the point location on fisheye image in central coordinates
    projectionPoint.x = xSign * xFocus * atan(sqrt(wx * wx + wy * wy) / wz)
        / sqrt((wy * wy) / (wx * wx) + 1);
    projectionPoint.y = ySign * yFocus * atan(sqrt(wx * wx + wy * wy) / wz)
        / sqrt((wx * wx) / (wy * wy) + 1);

    //  convert to corner coordinates
    toCorner(projectionPoint, newSize);

    return projectionPoint;
}

//cv::Mat AtanModel::projectPixelToWorld(cv::Point pixel)
//{
//
//}
