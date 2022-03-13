#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "vector"

// Stores parameters for the camera 
class Camera
{
	std::vector <double> polynom;		// Scaramuzza model coefficients
	cv::Vec2d centerOffset;				// Distortion center
	cv::Matx22d stretchMatrix;			// Whatever that is in the stretch matrix
	double lambda;						// Scale factor 
	/* Structures */
	cv::Mat map1;   // x map
	cv::Mat map2;   // y map
	std::vector<cv::Point> frameBorder; //border to draw on original image 


};

// On the second thought it is unnecessary - two fisheye dewarpers work as good. May be useful in a higher level class though. 