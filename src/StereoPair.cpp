#include "..\include\StereoPair.hpp"

Stereopair::Stereopair(std::shared_ptr<CameraModel> lCam, std::shared_ptr<FisheyeDewarper> lDWarp, 
                       std::shared_ptr<CameraModel> rCam, std::shared_ptr<FisheyeDewarper> rDWarp,
                       cv::Size outSize)
{
	leftCamera = lCam;
	rightCamera = rCam;

	leftDewarper = lDWarp;
	rightDewarper = rDWarp;

    outputSize = outSize;
}

Stereopair::Stereopair(std::shared_ptr<CameraModel> lCam, std::shared_ptr<FisheyeDewarper> lDWarp,
                       std::shared_ptr<CameraModel> rCam, std::shared_ptr<FisheyeDewarper> rDWarp,
                       cv::Size outSize, StereoMethod sm)
{
    leftCamera = lCam;
    rightCamera = rCam;

    leftDewarper = lDWarp;
    rightDewarper = rDWarp;

    outputSize = outSize;

    setStereoMethod(sm);
}

void Stereopair::fillMaps()
{
	leftDewarper->fillMaps();
	rightDewarper->fillMaps();
}

void Stereopair::setStereoMethod(StereoMethod sm)
{

    switch (sm)
    {
    case BM:
        matcher = cv::StereoBM::create();
        break;
    case SGBM:
        matcher = cv::StereoSGBM::create();
        break;
    default:
        break;
    }
}

void Stereopair::setOptimalDirecton()
{
	cv::Vec4d normilized;
	cv::normalize((leftCamera->rotation + rightCamera->rotation), normilized, 1.0, cv::NORM_L1);;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (normilized[3] * normilized[0] + normilized[1] * normilized[2]);
    double cosr_cosp = 1 - 2 * (normilized[0] * normilized[0] + normilized[1] * normilized[1]);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double pitch;
    double sinp = 2 * (normilized[3] * normilized[1] - normilized[2] * normilized[0]);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (normilized[3] * normilized[2] + normilized[0] * normilized[1]);
    double cosy_cosp = 1 - 2 * (normilized[1] * normilized[1] + normilized[2] * normilized[2]);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    leftDewarper->setRpy(  yaw, pitch, roll);   // FIXME:  wrong 
    rightDewarper->setRpy(-yaw, pitch, roll);
}

// get rectified?
int Stereopair::getRemapped(cv::Mat& left, cv::Mat& right, cv::Mat& leftRemapped, cv::Mat& rightRemapped)
{
    leftRemapped = leftDewarper->dewarpImage(left);
    rightRemapped = rightDewarper->dewarpImage(right);

    return 0;
}

int Stereopair::getDisparity(cv::OutputArray& dist, cv::InputArray& leftImage, cv::InputArray& rightImage)
{
    // static ?? 
    cv::Mat leftImageRemapped(leftCamera->newSize, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat rightImageRemapped(rightCamera->newSize, CV_8UC3, cv::Scalar(0, 0, 0));
    //
    getRemapped(leftImage.getMat(), rightImage.getMat(), leftImageRemapped, rightImageRemapped);
    matcher->compute(leftImage.getMat(), rightImage.getMat(), dist);

    return 0;
}

int Stereopair::getDepth(cv::OutputArray& dist, cv::InputArray& leftImage, cv::InputArray& rightImage)
{
    getDisparity(dist, leftImage, rightImage);

    // TODO: turn disparity into depth

    return 0;
}
