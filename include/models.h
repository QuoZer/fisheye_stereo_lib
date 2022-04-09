#pragma once 
#include "CameraModel.h"


class PinholeModel : public CameraModel
{
	/* Parameters as in parent */

public:	///* Projection functions *///
	PinholeModel();
	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Mat projectPixelToWorld(cv::Point pixel);
	void setIntrinsics(cv::Size newsize, float wideFov);
};


class ScaramuzzaModel : public CameraModel
{
private:	///* Intrinsics *///
	std::vector <double> scara_polynom;		// Scaramuzza model coefficients
	cv::Vec2d centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;			// Stretch matrix
	double lambda;						// Scale factor 

public:	///* Projection functions *///
	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Mat projectPixelToWorld(cv::Point pixel);

public:
	void setIntrinsics(std::initializer_list<double> coeffs, double lambda,
		cv::Vec2d centerOffset, cv::Matx22d stretchMatrix);
};

class AtanModel : public CameraModel
{
private:	///* Intrinsics *///
	cv::Vec2d centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;			// Stretch matrix
	double lambda;						// Scale factor 

public:	///* Projection functions *///
	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Point2d projectWorldToPixel2(cv::Mat worldPoint);
	cv::Mat projectPixelToWorld(cv::Point pixel) { return cv::Mat(oldSize, CV_8UC3, cv::Scalar(0, 0, 0)); }
	
public:
	void setIntrinsics(cv::Vec2d centerOffset, cv::Matx22d stretchMatrix, double scaleFactor) { return; };

};

class KBModel : public CameraModel
{
private:
	cv::Vec2d centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;			// Stretch matrix
	std::vector <double> kb_polynom;

public:
	KBModel();
	KBModel(std::initializer_list<double> coeffs,
		cv::Vec2d centerOffset, cv::Matx22d stretchMatrix);

	void setIntrinsics(std::initializer_list<double> coeffs,
		cv::Vec2d centerOffset, cv::Matx22d stretchMatrix);

	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Mat projectPixelToWorld(cv::Point pixel);
	void KBModel::backprojectSymmetric(cv::Point pxl, double& theta, double& phi);
};


class MeiModel : public CameraModel
{
private:
	double xi;
	double p1, p2;
	double k1, k2;
	std::vector<double> mei_polynom;

public:
	MeiModel();
	MeiModel(double xi, double p1, double p2, double k1, double k2, cv::Vec2d centerOffset, cv::Matx22d stretchMatrix);
	void setIntrinsics(double xi, double p1, double p2, double k1, double k2, cv::Vec2d centerOffset, cv::Matx22d stretchMatrix);
	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Mat MeiModel::projectPixelToWorld(cv::Point pixel);


};