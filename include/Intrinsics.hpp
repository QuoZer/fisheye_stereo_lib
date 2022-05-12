#pragma once
#include "opencv2/core.hpp"
#include <vector>

class DistortionParams
{
public:
	cv::Vec2d centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;			// Stretch matrix


};


class ScaraParams : public DistortionParams
{
public:
	std::vector <double> scara_polynom;		// Scaramuzza model coefficients
	double lambda;

	ScaraParams(std::initializer_list<double> coeffs,
		cv::Vec2d centerOffset, cv::Matx22d stretchMatrix, double scaleFactor)
	{
		this->scara_polynom.assign(coeffs.begin(), coeffs.end());       // treats both values as pointers 

		this->centerOffset = centerOffset;
		this->stretchMatrix = stretchMatrix;
		this->lambda = scaleFactor;
	}
};


class KBParams : public DistortionParams
{
public:
	std::vector <double> KB_polynom;		// KB model coefficients

	KBParams(std::initializer_list<double> coeffs,
		cv::Vec2d centerOffset, cv::Matx22d stretchMatrix)
	{
		this->KB_polynom.assign(coeffs.begin(), coeffs.end());       // treats both values as pointers 
		this->centerOffset = centerOffset;
		this->stretchMatrix = stretchMatrix;
	}
};