
#include "StereoPair.hpp"

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

std::vector<cv::Mat> Stereopair::getMaps()
{
    std::vector<cv::Mat> leftMap = leftDewarper->getMaps();
    std::vector<cv::Mat> rightMap = rightDewarper->getMaps();
    // merging the vectors
    leftMap.insert(leftMap.end(), rightMap.begin(), rightMap.end());

    return leftMap;
}

void Stereopair::saveMaps(int index, std::string& systemId)
{
    std::vector<cv::Mat> leftMap = leftDewarper->getMaps();
    std::vector<cv::Mat> rightMap = rightDewarper->getMaps();

    leftMap.insert(leftMap.end(), rightMap.begin(), rightMap.end());

    cv::Mat full_map;
    cv::merge(leftMap, full_map);

    std::string name = systemId + std::to_string(index) + "_" + leftCamera->modelName + rightCamera->modelName + ".png";

    cv::imwrite(name, full_map);        // TODO: casts the image to 8 bit, breaks the maps

}

void Stereopair::loadMaps(int index, const char* systemId)
{
    std::string filename = systemId + std::to_string(index) + "_" + leftCamera->modelName + rightCamera->modelName + ".png";

    cv::Mat full_map;
    full_map = cv::imread(filename, cv::IMREAD_UNCHANGED);    // TODO: safety checks

    std::vector<cv::Mat> fourChannels;
    cv::split(full_map, fourChannels);

    leftDewarper->setMaps(fourChannels[0], fourChannels[1]);
    rightDewarper->setMaps(fourChannels[2], fourChannels[3]);
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

// return a number's sign
int Stereopair::sign(double x)
{
	if (x > 0)
		return 1;
	else if (x < 0)
		return -1;
	else
		return 0;
}

void Stereopair::setOptimalDirecton()
{
    cv::Vec3d l_rpy = quatToRpy(leftCamera->rotation);
    cv::Vec3d r_rpy = quatToRpy(rightCamera->rotation);

    // 2D case for now 
    double l_z = l_rpy[0];  //yaw
	double r_z = r_rpy[0];

	double dir_z = (l_z + r_z) / 2;
	
    if (std::abs(l_z) + std::abs(r_z) > 3.14 && std::min(l_z, r_z) < 0 && std::max(l_z, r_z) > 0)     // not sure
    {
		//lower half 
        dir_z += 3.14;		
    }
	
	l_rpy[0] = (l_z - dir_z);
	r_rpy[0] = (r_z - dir_z);	

    leftDewarper->setRpyRad(l_rpy);   // TODO: FIXME:  wrong 
    rightDewarper->setRpyRad(r_rpy);
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
    
    //getRemapped(leftImage.getMat(), rightImage.getMat(), leftImageRemapped, rightImageRemapped);
    
    // TODO: check if the matcher is ready before computing
    matcher->compute(leftImageRemapped, rightImageRemapped, dist);

    return 0;
}

int Stereopair::getDepth(cv::OutputArray& dist, cv::InputArray& leftImage, cv::InputArray& rightImage)
{
    getDisparity(dist, leftImage, rightImage);

    // TODO: turn disparity into depth

    return 0;
}
