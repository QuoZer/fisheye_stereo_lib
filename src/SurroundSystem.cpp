#include "SurroundSystem.hpp"

// takes cameramodel object with all the parameters already set. Return new camera index
int SurroundSystem::addNewCam(CameraModel& readyModel)
{
	std::shared_ptr<CameraModel> sharedModel(&readyModel);
	cameras.push_back(sharedModel);
	dewarpers.push_back(std::shared_ptr<FisheyeDewarper>( new FisheyeDewarper(sharedModel) ));

	return cameras.size()-1;  // new camera index
}

//CameraModel SurroundSystem::getCameraModel(CameraModels cm)
//{
//	// FIXME: antipattern
//	switch (cm)
//	{
//	case PINHOLE:
//		return PinholeModel();
//	case ATAN:
//		return AtanModel();
//	case SCARAMUZZA:
//		return ScaramuzzaModel();
//	case MEI:
//		return MeiModel();
//	case KB:
//		break;
//	default:
//		break;
//	}
//}

int SurroundSystem::createStereopair(int lCamIndex, int rCamIndex, cv::Size reconstructedRes, cv::Vec3d direction, StereoMethod sm)
{
	std::shared_ptr<CameraModel> left = cameras[lCamIndex];
	std::shared_ptr<CameraModel> right = cameras[rCamIndex];
	std::shared_ptr <FisheyeDewarper> leftDewarper = dewarpers[lCamIndex];
	std::shared_ptr <FisheyeDewarper> rightDewarper = dewarpers[rCamIndex];
	// TODO: interpolation setter ?? 
	leftDewarper->setSize(left->oldSize, reconstructedRes, 90);  // HACK: 90deg is an assumption
	leftDewarper->setRpy(0, 0, 0);									// TODO: calc angles from position
	rightDewarper->setSize(right->oldSize, reconstructedRes, 90);
	rightDewarper->setRpy(0, 0, 0);

	std::shared_ptr<Stereopair> SP( new Stereopair(left, leftDewarper, right, rightDewarper, reconstructedRes) );
	SP->setOptimalDirecton();
	SP->setStereoMethod(sm);
	stereopairs.push_back(SP);



	return stereopairs.size() - 1;
}

int SurroundSystem::createStereopair(const CameraModel& leftModel, const CameraModel& rightModel, cv::Size reconstructedRes, cv::Vec3d direction, StereoMethod sm)
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
	SP->setOptimalDirecton();
	SP->setStereoMethod(sm);
	stereopairs.push_back(SP);

	return stereopairs.size() - 1;
}

int SurroundSystem::loadLUTs()
{
	int index = 0;
	int result = 1;
	for (auto SP : stereopairs)
	{
		index++;
		result *= SP->loadMaps(std::to_string(index));
	}

	return result;
}

void SurroundSystem::prepareLUTs(bool saveResults)
{
	int index = 0;
	for (auto SP : stereopairs)
	{
		index++;
		SP->fillMaps();
		if (saveResults)
			SP->saveMaps(std::to_string(index));
	}
}

// TODO: delete r and l - use constant image pointers instead?
void SurroundSystem::getImage(int stereopairIndex, ImageType IT, cv::Mat& l, cv::Mat& r, cv::Mat& dst)
{
	cv::Size out_size = stereopairs[stereopairIndex]->outputSize;
	cv::Mat leftImageRemapped(out_size, CV_8UC3, cv::Scalar(0, 0, 0));
	cv::Mat rightImageRemapped(out_size, CV_8UC3, cv::Scalar(0, 0, 0));	// TODO: rework 'size' system
	switch (IT)
	{
	case SurroundSystem::RAW:		// why the hell would i need that?
		break;
	case SurroundSystem::RECTIFIED:
		stereopairs[stereopairIndex]->getRemapped(l, r, leftImageRemapped, rightImageRemapped) ;
		cv::hconcat(leftImageRemapped, rightImageRemapped, dst);
		return;
	case SurroundSystem::DISPARITY:
		stereopairs[stereopairIndex]->getDisparity(dst, l, r);
		return;
	case SurroundSystem::DEPTH:
		stereopairs[stereopairIndex]->getDepth(dst, l, r);
		return;
	default:
		break;
	}

	
}
