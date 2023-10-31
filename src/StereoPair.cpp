
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

void Stereopair::loadStereoSGBMParameters(const std::string& filename, cv::Ptr<cv::StereoSGBM>& matcher) 
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Error: Could not open the parameter file " << filename << std::endl;
        return;
    }

    // Load parameters from the file
    // Assign the loaded parameters to the matcher
    matcher = cv::StereoSGBM::create(fs["minDisparity"], (int)fs["numDisparities"]*16, (int)fs["blockSize"]*2+5);
    matcher->setPreFilterCap(fs["preFilterCap"]);
    matcher->setUniquenessRatio(fs["uniquenessRatio"]);
    matcher->setSpeckleWindowSize((int)fs["speckleWindowSize"]*2);
    matcher->setSpeckleRange(fs["speckleRange"]);
    matcher->setDisp12MaxDiff(fs["disp12MaxDiff"]);
    //preFilterType : 1
    //preFilterSize : 1
    //textureThreshold : 10
    fs.release();
}

void Stereopair::setStereoMethod(StereoMethod sm, std::string params_path)
{
    if ( (sm == SGBM || sm == BM) && params_path.empty())
		throw std::runtime_error("No parameter file assigned");
    if (sm == SGBM)
		matcher = cv::StereoSGBM::create();
	else if (sm == BM)
		matcher = cv::StereoBM::create();
	else
		throw std::runtime_error("No matcher assigned");

    //matcher = cv::StereoSGBM::create();
    cv::Ptr<cv::StereoSGBM> sgbm;
    loadStereoSGBMParameters(params_path, sgbm);
    matcher = sgbm;

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
    cv::Mat leftImageRemapped(this->outputSize, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat rightImageRemapped(this->outputSize, CV_8UC3, cv::Scalar(0, 0, 0));
    
    cv::cvtColor(leftImage, leftImageRemapped, cv::COLOR_BGR2GRAY);
    cv::cvtColor(rightImage, rightImageRemapped, cv::COLOR_BGR2GRAY);

    cv::Mat disp, disparity;
    // TODO: check if the matcher is ready before computing
    matcher->compute(leftImageRemapped, rightImageRemapped, disp);

    disp.convertTo(disparity, CV_32F, 1.0);
    disparity = (disparity / 16.0f - (float)matcher->getMinDisparity()) / ((float)matcher->getNumDisparities());
    disparity.copyTo(dist);

    return 0;
}

int Stereopair::getDepth(cv::OutputArray& dist, cv::InputArray& leftImage, cv::InputArray& rightImage)
{
    getDisparity(dist, leftImage, rightImage);

    // TODO: turn disparity into depth

    return 0;
}
