#pragma once 
#include "CameraModel.h"

/*
	The file for defining camera models. Write or #include yours here
*/

/// <summary>
/// Basic pinhole model
/// </summary>
class PinholeModel : public CameraModel
{
	/* Parameters as in the parent */

public:	///* Projection functions *///
	PinholeModel();
	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Mat projectPixelToWorld(cv::Point pixel);
	void setIntrinsics(cv::Size newsize, float wideFov);
};

/// <summary>
/// Scaramuzza camera model from the MATLAB camera calibration toolbox. 
/// Distortion parameters: 4 coefficients and a scaling factor.
/// Projection parameters: image centre coordinates and a stretch matrix.
/// More at: https://www.mathworks.com/help/vision/ug/fisheye-calibration-basics.html
/// </summary>
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
	/// <summary>
	/// Specify camera intrinsic parameters according to the model. Can be obtained from e.g. MATLAB calibration toolbox. 
	/// </summary>
	/// <param name="coeffs"> 4 polynomial coefficients as a list in '{}' brackets.  </param>
	/// <param name="lambda"> Lambda scaling factor </param>
	/// <param name="centerOffset"> Image centre </param>
	/// <param name="stretchMatrix"> 2x2 Stretch matrix </param>
	void setIntrinsics(std::initializer_list<double> coeffs, double lambda,
		cv::Vec2d centerOffset, cv::Matx22d stretchMatrix);
};

/// <summary>
/// 
/// </summary>
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
	void setIntrinsics(cv::Vec2d centerOffset, cv::Matx22d stretchMatrix);

};

class RealAtanModel : public CameraModel
{
private:	///* Intrinsics *///
	cv::Vec2d centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;			// Stretch matrix
	double lambda;						// Scale factor 

public:	///* Projection functions *///
	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Mat projectPixelToWorld(cv::Point pixel) { return cv::Mat(oldSize, CV_8UC3, cv::Scalar(0, 0, 0)); }

public:
	void setIntrinsics(cv::Vec2d centerOffset, cv::Matx22d stretchMatrix);

};

/// <summary>
/// Kannala-Brandt camera model. Can be calibrated in OpenCV and CamOdoCal libraries
/// Distortion parameters: 4 coefficients.
/// Projection parameters: image centre coordinates and a stretch matrix.
/// More at:
/// </summary>
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

	/// <summary>
	/// Specify camera intrinsic parameters according to the model.
	/// </summary>
	/// <param name="coeffs"> 4 polynomial coefficients as a list in '{}' brackets. </param>
	/// <param name="centerOffset"> Image centre </param>
	/// <param name="stretchMatrix"> 2x2 Stretch matrix </param>
	void setIntrinsics(std::initializer_list<double> coeffs,
		cv::Vec2d centerOffset, cv::Matx22d stretchMatrix);

	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Mat projectPixelToWorld(cv::Point pixel);

	// reference
	void backprojectSymmetric(cv::Point pxl, double& theta, double& phi);
};

/// <summary>
/// Christopher Mei camera model. Can be calibrated using CamOdoCal library
/// Distortion parameters: 5 coefficients.
/// Projection parameters: image centre coordinates and a stretch matrix.
/// More at:
/// </summary>
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
	cv::Mat projectPixelToWorld(cv::Point pixel);
};

/// <summary>
/// Double sphere model (doesn't seem to work at the moment)
/// Distortion parameters: 2 coefficients.
/// Projection parameters: image centre coordinates and a stretch matrix.
/// More at:
/// </summary>
class DSModel : public CameraModel
{
private:
	double alpha;
	double dzeta;

public:
	DSModel();
	DSModel(double alpha, double dzeta, cv::Vec2d centerOffset, cv::Matx22d stretchMatrix);
	void setIntrinsics(double alpha, double dzeta, cv::Vec2d centerOffset, cv::Matx22d stretchMatrix);
	cv::Point2d projectWorldToPixel(cv::Mat worldPoint);
	cv::Mat projectPixelToWorld(cv::Point pixel);
};