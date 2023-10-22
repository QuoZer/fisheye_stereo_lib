#include "SurroundSystem.hpp"


SurroundSystem::SurroundSystem(const char* systemName)
{
	this->systemName = systemName; 
}

int SurroundSystem::addNewCam(CameraModel& readyModel)
{
	std::shared_ptr<CameraModel> sharedModel(&readyModel);
	cameras.push_back(sharedModel);
	//dewarpers.push_back(std::shared_ptr<FisheyeDewarper>( new FisheyeDewarper(sharedModel) ));

	return cameras.size()-1;  // new camera index
}


int SurroundSystem::createStereopair(int lCamIndex, int rCamIndex, cv::Size reconstructedRes, cv::Vec3d direction, StereoMethod sm, const std::string& stereoParamsPath )
{
	std::shared_ptr<CameraModel> left = cameras[lCamIndex];
	std::shared_ptr<CameraModel> right = cameras[rCamIndex];
	// create and save dewarpers
	std::shared_ptr <FisheyeDewarper> leftDewarper(new FisheyeDewarper(left));
	dewarpers.push_back(leftDewarper);
	std::shared_ptr <FisheyeDewarper> rightDewarper(new FisheyeDewarper(right));
	dewarpers.push_back(rightDewarper);
	// TODO: interpolation setter ?? 
	leftDewarper->setSize(left->oldSize, reconstructedRes, 90);  // HACK: pinhole 90deg fow is an assumption
	rightDewarper->setSize(right->oldSize, reconstructedRes, 90);
	// create a stereopair based on the newly created dewarpers
	std::shared_ptr<Stereopair> SP( new Stereopair(left, leftDewarper, right, rightDewarper, reconstructedRes) );
	SP->setOptimalDirecton();
	SP->setStereoMethod(sm, stereoParamsPath);
	stereopairs.push_back(SP);
	// return index
	return stereopairs.size() - 1;
}

void SurroundSystem::readCamera(cv::FileNode& node) 
{
	std::string camera_model = node["model"];
	if (camera_model == "Scaramuzza")
	{
		ScaramuzzaModel newScaraCamera;
		//std::initializer_list <double> coefficients;
		cv::Vec4d coefficients;
		cv::Vec2d principal_point;
		cv::Matx22d skew;
		cv::Vec3d pos;
		cv::Vec4d rot;
		cv::Size original(node["intrinsics"]["resolution"]["width"], node["intrinsics"]["resolution"]["height"]);
		node["extrinsics"]["translation"] >> pos;
		node["extrinsics"]["rotation"] >> rot;
		node["intrinsics"]["skew"] >> skew;
		node["intrinsics"]["principal_point"] >> principal_point;
		node["intrinsics"]["coefficients"] >> coefficients;
		newScaraCamera.setIntrinsics(coefficients, node["intrinsics"]["lambda"], principal_point, skew);
		newScaraCamera.setExtrinsics(pos, rot);
		newScaraCamera.setCamParams(original);

		this->addNewCam(newScaraCamera);
	}
	if (camera_model == "KB")
	{
		KBModel newKBCamera;
		cv::Vec4d coefficients;
		cv::Vec2d principal_point;
		cv::Matx22d skew;
		cv::Vec3d pos;
		cv::Vec4d rot;
		cv::Size original(node["intrinsics"]["resolution"]["width"], node["intrinsics"]["resolution"]["height"]);
		node["extrinsics"]["translation"] >> pos;
		node["extrinsics"]["rotation"] >> rot;
		node["intrinsics"]["skew"] >> skew;
		node["intrinsics"]["principal_point"] >> principal_point;
		node["intrinsics"]["coefficients"] >> coefficients;
		newKBCamera.setIntrinsics(coefficients, principal_point, skew);
		newKBCamera.setExtrinsics(pos, rot);
		newKBCamera.setCamParams(original);

		this->addNewCam(newKBCamera);
	}
	// TODO: add other camera models
}

void SurroundSystem::readStereopair(cv::FileNode& node)
{
	cv::FileNode sp = node;
	cv::Size out_size(sp["out_resolution"]["width"], sp["out_resolution"]["height"]);
	cv::Vec3d direction;
	std::string stereo_method_str;
	sp["orientation"] >> direction;
	sp["method"] >> stereo_method_str;

	StereoMethod stereo_method;
	if (stereo_method_str == "BM") stereo_method = StereoMethod::BM;
	else if (stereo_method_str == "SGBM") stereo_method = StereoMethod::SGBM;
	else std::cerr << "Unknown stereo method " << stereo_method_str << std::endl;

	this->createStereopair((int)sp["camera1"], (int)sp["camera2"], out_size, direction, stereo_method, sp["parameters_file"]);
}

void SurroundSystem::readSystemParams(const std::string& filepath)
{
	cv::FileStorage fs(filepath, cv::FileStorage::READ);
	if (!fs.isOpened()) {
		std::cerr << "Failed to open file " << filepath << std::endl;
		return ;
	}
	

	cv::FileNode cameras_node = fs["system"]["cameras"];
	for (cv::FileNodeIterator it = cameras_node.begin(); it != cameras_node.end(); ++it) 
	{
		readCamera(*it);
	}

	cv::FileNode stereo_pairs_node = fs["system"]["stereopairs"];
	for (cv::FileNodeIterator it = stereo_pairs_node.begin(); it != stereo_pairs_node.end(); ++it) 
	{
		readStereopair(*it);
	}

}

int SurroundSystem::createStereopair(CameraModel& leftModel, CameraModel& rightModel, cv::Size reconstructedRes, cv::Vec3d direction, StereoMethod sm, const std::string& stereoParamsPath)
{
	int lCamIndex = addNewCam(leftModel);
	int rCamIndex = addNewCam(rightModel);
	std::shared_ptr<CameraModel> left = cameras[lCamIndex];
	std::shared_ptr<CameraModel> right = cameras[rCamIndex];
	std::shared_ptr<FisheyeDewarper> leftDewarper = dewarpers[lCamIndex];
	std::shared_ptr<FisheyeDewarper> rightDewarper = dewarpers[rCamIndex];
	// TODO: interpolation setter ?? 
	leftDewarper->setSize(left->oldSize, reconstructedRes, 90);  // HACK: 90deg is an assumption
	leftDewarper->setRpy(0, 0, 0);									
	rightDewarper->setSize(right->oldSize, reconstructedRes, 90);
	rightDewarper->setRpy(0, 0, 0);

	std::shared_ptr<Stereopair> SP( new Stereopair(left, leftDewarper, right, rightDewarper, reconstructedRes) );
	//SP->setOptimalDirecton();
	SP->setStereoMethod(sm, stereoParamsPath);
	stereopairs.push_back(SP);

	return stereopairs.size() - 1;
}

int SurroundSystem::loadLUTs(const char* name)	// we don't actually need a name, since SS already has one
{
	int index = 0;
	int result = 1;
	for (auto SP : stereopairs)
	{
		SP->loadMaps(index, name);
		index++;
	}

	return result;
}



void SurroundSystem::prepareLUTs(bool saveResults)
{
	int index = 0;
	
	std::vector<cv::Mat> all_maps;

	for (auto SP : stereopairs)
	{
		std::cout << "Prepping SP" << index << std::endl;
		SP->fillMaps();
		if (saveResults) {
			SP->saveMaps(index, this->systemName);
		}
		index++;
	}
	
}

int SurroundSystem::getNumOfSP()
{
	return stereopairs.size();
}

// TODO: delete r and l - use constant image pointers / streams instead?
void SurroundSystem::getImage(int stereopairIndex, ImageType IT, cv::Mat& l, cv::Mat& r, cv::Mat& dst)
{
	if (stereopairIndex >= stereopairs.size())
		return;
	
	// TODO: check if the index is valid
	cv::Size out_size = stereopairs[stereopairIndex]->outputSize;
	cv::Mat leftImageRemapped(out_size, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat rightImageRemapped(out_size, CV_8UC3, cv::Scalar(0, 0, 0));	

	switch (IT)
	{
	case SurroundSystem::RAW:		// why the hell would I need that?
		break;
	case SurroundSystem::RECTIFIED:
		stereopairs[stereopairIndex]->getRemapped(l, r, leftImageRemapped, rightImageRemapped) ;
		leftImageRemapped = leftImageRemapped(cv::Rect(0, 0, out_size.width, out_size.height));
		rightImageRemapped = rightImageRemapped(cv::Rect(0, 0, out_size.width, out_size.height));
		cv::hconcat(leftImageRemapped, rightImageRemapped, dst);
		return;
	case SurroundSystem::DISPARITY:
		stereopairs[stereopairIndex]->getRemapped(l, r, leftImageRemapped, rightImageRemapped);
		leftImageRemapped = leftImageRemapped(cv::Rect(0, 0, out_size.width, out_size.height));
		rightImageRemapped = rightImageRemapped(cv::Rect(0, 0, out_size.width, out_size.height));
		stereopairs[stereopairIndex]->getDisparity(dst, leftImageRemapped, rightImageRemapped);
		return;
	case SurroundSystem::DEPTH:
		stereopairs[stereopairIndex]->getDepth(dst, l, r);
		return;
	default:
		break;
	}

	
}
