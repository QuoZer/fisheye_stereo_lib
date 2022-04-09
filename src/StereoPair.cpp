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

cv::Vec3d Stereopair::quatToRpy(cv::Vec4d quaternion)
{
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (quaternion[3] * quaternion[0] + quaternion[1] * quaternion[2]);
    double cosr_cosp = 1 - 2 * (quaternion[0] * quaternion[0] + quaternion[1] * quaternion[1]);
    double roll = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double pitch;
    double sinp = 2 * (quaternion[3] * quaternion[1] - quaternion[2] * quaternion[0]);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (quaternion[3] * quaternion[2] + quaternion[0] * quaternion[1]);
    double cosy_cosp = 1 - 2 * (quaternion[1] * quaternion[1] + quaternion[2] * quaternion[2]);
    double yaw = std::atan2(siny_cosp, cosy_cosp);
    
    
    return cv::Vec3d(yaw, pitch, roll);
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
    cv::Vec3d l_rpy = quatToRpy(leftCamera->rotation);
    cv::Vec3d r_rpy = quatToRpy(rightCamera->rotation);
    
    leftDewarper->setRpyRad(r_rpy);   // FIXME:  wrong 
    rightDewarper->setRpyRad(l_rpy);
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
    matcher->compute(leftImageRemapped, rightImageRemapped, dist);

    return 0;
}

int Stereopair::getDepth(cv::OutputArray& dist, cv::InputArray& leftImage, cv::InputArray& rightImage)
{
    getDisparity(dist, leftImage, rightImage);

    // TODO: turn disparity into depth

    return 0;
}
