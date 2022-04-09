#include "models.h"

// equidistant fisheye 
cv::Point2d AtanModel::projectWorldToPixel2(cv::Mat worldPoint)
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
    double xFocus = oldSize.width / M_PI;       // TODO: check the logic begind this size
    double yFocus = oldSize.height / M_PI;
    cv::Point projectionPoint(oldSize.width / 2, oldSize.height / 2);       //  initial value set to the image corner

    //  calculate the point location on fisheye image in central coordinates
    projectionPoint.x = xSign * xFocus * atan(sqrt(wx * wx + wy * wy) / wz)
        / sqrt((wy * wy) / (wx * wx) + 1);
    projectionPoint.y = ySign * yFocus * atan(sqrt(wx * wx + wy * wy) / wz)
        / sqrt((wx * wx) / (wy * wy) + 1);

    //  convert to corner coordinates
    toCorner(projectionPoint, oldSize);

    return projectionPoint;
}

 

cv::Point2d AtanModel::projectWorldToPixel(cv::Mat worldPoint)
{
    double X = worldPoint.at<float>(0);
    double Y = worldPoint.at<float>(1);
    double Z = worldPoint.at<float>(2);
    double phi = atan2( sqrt(X * X + Y * Y), Z);
    double rho = 0;
    double r = 0;
    double error = 1;

    //int iter = 0;
    //do
    //{
    //    double R = cos(r);

    //    error = atan2(R, r) - phi;
    //    r = r + 1.15 * error;
    //    iter++;
    //    //std::cout << "It " << iter << " Point: " << worldPoint << " | Rho: " << rho << " f(Rho): " << 424242 << " Error: " << error << std::endl;
    //} while (std::abs(error) > 0.005 && iter < 100);
    //this->errorsum += error;

    double xFocus = 1 * oldSize.width / M_PI;       
    double yFocus = 1 * oldSize.height / M_PI;
    double u = xFocus * (phi) / sqrt((Y * Y) / (X * X) + 1);
    double v = yFocus * (phi) / sqrt((X * X) / (Y * Y) + 1);

    cv::Point fypixel(stretchMatrix * cv::Vec2d(u, v) + centerOffset);        // technically could do toCorner's job, but I'll keep it simple for now
    toCorner(fypixel, oldSize);
    return fypixel;
}

void AtanModel::setIntrinsics( cv::Vec2d centerOffset, cv::Matx22d stretchMatrix)
{
    this->centerOffset = centerOffset;
    this->stretchMatrix = stretchMatrix;
}
