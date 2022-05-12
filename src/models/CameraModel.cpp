#include "CameraModel.h"


CameraModel::CameraModel()
{
	errorsum = 0;
}

void CameraModel::toCenter(cv::Point& cornerPixel, cv::Size imagesize)
{
	cornerPixel.x = cornerPixel.x - imagesize.width / 2;
	cornerPixel.y = -cornerPixel.y + imagesize.height / 2;
}

void CameraModel::toCorner(cv::Point& centerPixel, cv::Size imagesize)
{
	centerPixel.x = centerPixel.x + imagesize.width / 2;
	centerPixel.y = -centerPixel.y + imagesize.height / 2;
}

void CameraModel::setExtrinsics(cv::Vec3d pos, cv::Vec4d rot)
{
	position = pos;
	rotation = rot;
}

void CameraModel::setCamParams(cv::Size origImageSize)
{
	oldSize = origImageSize;
}