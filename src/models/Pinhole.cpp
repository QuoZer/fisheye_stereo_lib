#include "models.h"


cv::Point2d PinholeModel::projectWorldToPixel(cv::Mat worldPoint)
{
    double xFovRad = xFov * M_PI / 180;
    double yFovRad = yFov * M_PI / 180;
    double xPinholeFocus = newSize.width / (2 * tan(xFovRad / 2));
    double yPinholeFocus = newSize.height / (2 * tan(yFovRad / 2));

    float cx = worldPoint.at<float>(0);
    float cy = worldPoint.at<float>(1);
    float cz = worldPoint.at<float>(2);

    cv::Point2f pinholePoint;
    pinholePoint.x = xPinholeFocus * cx / cz;
    pinholePoint.y = yPinholeFocus * cy / cz;

    return pinholePoint;
}

cv::Mat PinholeModel::projectPixelToWorld(cv::Point pixel)
{
    toCenter(pixel, newSize);
    //std::cout << pixel << " | " << newSize << std::endl;

    float cz = 10;                                    // doesnt really affect much
    double xFovRad = xFov * M_PI / 180;
    double yFovRad = yFov * M_PI / 180;
    double xPinholeFocus = newSize.width / (2 * tan(xFovRad / 2));
    double yPinholeFocus = newSize.height / (2 * tan(yFovRad / 2));

    cv::Mat cameraCoords(1, 3, CV_32F, float(0));
    cameraCoords.at<float>(0) = pixel.x * cz / xPinholeFocus;
    cameraCoords.at<float>(1) = pixel.y * cz / yPinholeFocus;
    cameraCoords.at<float>(2) = cz;
    //std::cout << xPinholeFocus << " | " << yPinholeFocus << std::endl;
    return cameraCoords;
}

void PinholeModel::setIntrinsics(cv::Size newsize, float wideFov)
{
    newSize = newsize;
    xFov = wideFov;
    yFov = wideFov * newSize.height / newSize.width;           // * 9/16 - vertical fov
}

PinholeModel::PinholeModel()
{
    errorsum = 0;
}
