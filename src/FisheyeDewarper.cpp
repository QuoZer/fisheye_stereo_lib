#include "FisheyeDewarper.hpp"

#define CVPLOT_HEADER_ONLY
//#include <CvPlot/cvplot.h>

FisheyeDewarper::FisheyeDewarper()
{
    oldSize = cv::Size(0, 0);
    newSize = cv::Size(0, 0);
    roll = 0;
    pitch = 0;
    yaw = 0;
    errorsum = 0;
    xFov = 60.0;
    yFov = 6.0; // idk
    cameraModel = nullptr;
    pinhole = nullptr;

}

// https://stackoverflow.com/questions/30069384/provides-no-initializer-for-reference-member
FisheyeDewarper::FisheyeDewarper(std::shared_ptr<CameraModel> model) : FisheyeDewarper()
{
    cameraModel = model;
    pinhole = std::shared_ptr<PinholeModel>(new PinholeModel());
    oldSize = model->oldSize;
}


void FisheyeDewarper::createMaps()
{
    map1 = cv::Mat(oldSize, CV_32FC1, float(0));
    map2 = cv::Mat(oldSize, CV_32FC1, float(0));
}

void FisheyeDewarper::setRpy(cv::Vec3d angles)
{
    this->yaw = angles[0] * PI / 180;
    this->pitch = -angles[1] * PI / 180;
    this->roll = angles[2] * PI / 180;
}

void FisheyeDewarper::setRpy(float yaw, float pitch, float roll)
{
    this->yaw = yaw * PI / 180;
    this->pitch = -pitch * PI / 180;
    this->roll = roll * PI / 180;

    // TODO: fillMaps() after angle update. Or should it be handled manually? 
}

void FisheyeDewarper::setRpyRad(cv::Vec3d rad_angles)
{
    this->yaw =   rad_angles[0];
    this->pitch = rad_angles[1];
    this->roll =  rad_angles[2];
}

void FisheyeDewarper::setSize(cv::Size oldsize, cv::Size newsize, float wideFov)
{
    this->oldSize = oldsize;
    this->newSize = newsize;
    //std::cout << "Pinhole parameters set" << pinhole.errorsum << std::endl;
    pinhole->setIntrinsics( newSize, wideFov);
    //createMaps();
}

std::vector<cv::Point> FisheyeDewarper::getBorder() {
    return this->frameBorder;
}

cv::Mat FisheyeDewarper::rotatePoint(cv::Mat worldPoint)
{
    cv::Mat rotZ(cv::Matx33f(1, 0, 0,
        0, cos(yaw), sin(yaw),
        0, -sin(yaw), cos(yaw)));
    cv::Mat rotX(cv::Matx33f(cos(pitch), 0, -sin(pitch),
        0, 1, 0,
        sin(pitch), 0, cos(pitch)));
    cv::Mat rotY(cv::Matx33f(cos(roll), -sin(roll), 0,
        sin(roll), cos(roll), 0,
        0, 0, 1));
    return worldPoint * rotY * rotZ * rotX;         // calib3d/utils  proposes this order
}

// converting corner coordinates to the center ones
void FisheyeDewarper::toCenter(cv::Point& cornerPixel, cv::Size imagesize)
{
    cornerPixel.x = cornerPixel.x - imagesize.width / 2;
    cornerPixel.y = -cornerPixel.y + imagesize.height / 2;
}

// converting center coordinates to the corner ones
void FisheyeDewarper::toCorner(cv::Point& centerPixel, cv::Size imagesize)
{
    centerPixel.x = centerPixel.x + imagesize.width / 2;
    centerPixel.y = -centerPixel.y + imagesize.height / 2;
}

void FisheyeDewarper::fillMaps()
{
    createMaps();
    frameBorder.clear();
	
    std::vector<float> points(newSize.width);

    double max_xy = 0;
    for (int i = 0; i < newSize.width; i++)
    {
        for (int j = 0; j < newSize.height; j++)
        {
            cv::Mat worldPoint = pinhole->projectPixelToWorld(cv::Point(i, j));
            worldPoint = rotatePoint(worldPoint);
            cv::Point distPoint = cameraModel->projectWorldToPixel(worldPoint);
            if (distPoint.x > oldSize.width - 1 || distPoint.x < 0 ||
                distPoint.y > oldSize.height - 1 || distPoint.y < 0)
            {
                continue;   // skips out of border points
            }

            // save distorted edge of the frame 
            if (((j == 0 || j == newSize.height - 1) && i % 100 == 0) ||
                ((i == 0 || i == newSize.width - 1) && j % 100 == 0))
            {
                frameBorder.push_back(cv::Point(distPoint.y, distPoint.x));
            }
            // save the z value of each point
            if (worldPoint.at<float>(0) > max_xy || worldPoint.at<float>(1) > max_xy) {
				max_xy = std::max(worldPoint.at<float>(0), worldPoint.at<float>(1));
            }

            map1.at<float>(i, j) = distPoint.y;
            map2.at<float>(i, j) = distPoint.x;
        }
        //if (i % 100 == 0) std::cout << "Collumn N" << i << std::endl;
    }
    //std::cout << "Avg. error: " << cameraModel->errorsum / (newSize.area()) << std::endl;

}

void FisheyeDewarper::saveMaps(std::string prefix)
{
    std::vector<cv::Mat> channels = {map1, map2};
    cv::Mat full_map;

    cv::merge(channels, full_map);

    cv::imwrite(prefix + "_map.png", full_map);
}

int FisheyeDewarper::loadMaps(std::string prefix)
{
    std::vector<cv::Mat> channels;
    cv::Mat combined_map = cv::imread(prefix + "_map.png");
    if (combined_map.empty())
    {
        return 0;
    }
    cv::split(combined_map, channels);

    this->map1 = channels[0];
    this->map2 = channels[1];
    return 1;
}

cv::Mat FisheyeDewarper::dewarpImage(cv::Mat inputImage)
{
    cv::Mat remapped(newSize, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::remap(inputImage, remapped, map1, map2, cv::INTER_NEAREST, cv::BORDER_CONSTANT);
    return remapped;
}