#include "models.hpp"


DSModel::DSModel(double alpha, double dzeta, cv::Vec2d centerOffset, cv::Matx22d stretchMatrix)
{
    setIntrinsics( alpha,  dzeta,  centerOffset,  stretchMatrix);
    setModelName("DS");
}

DSModel::DSModel()
{
    alpha = 0;
    dzeta = 0;
    setModelName("DS");
}

// equidistant fisheye 
cv::Point2d DSModel::projectWorldToPixel(cv::Mat worldPoint)
{
    float wx = worldPoint.at<float>(0);
    float wy = worldPoint.at<float>(1);
    float wz = worldPoint.at<float>(2);
    double d_one = sqrt(wx*wx + wy*wy + wz*wz);
    double d_two = sqrt(wx*wx + wy*wy + pow((this->dzeta * d_one + wz), 2) );

    double znam = alpha * d_two + (1 - this->alpha) * (this->dzeta * d_one + wz);
    //  calculate the point location on fisheye image in central coordinates
    double u = wx / znam;
    double v = wy / znam;
    cv::Point fypixel(stretchMatrix * cv::Vec2d(u, -v) + centerOffset);
    //  convert to corner coordinates
    //toCorner(fypixel, oldSize);

    return fypixel;
}

cv::Mat DSModel::projectPixelToWorld(cv::Point pixel)
{

    cv::Mat cameraCoords(1, 3, CV_32F, float(0));

    return cameraCoords;
}

void DSModel::setIntrinsics(double alpha, double dzeta, cv::Vec2d centerOffset, cv::Matx22d stretchMatrix)
{
    this->alpha = alpha;
    this->dzeta = dzeta;
    this->centerOffset = centerOffset;
    this->stretchMatrix = stretchMatrix;
}