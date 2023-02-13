#include "opencv2/ccalib/omnidir.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/videoio.hpp>
#include <vector>
#include <iostream>
#include <string>
#include <time.h>
#include <map>
#include <cstdarg>
#include "SurroundSystem.hpp"
#include "Utils.hpp"

// -w=9 -h=6 -pt=chessboard ..\..\data\4_Compar0.1m\stereo_list.xml

const bool DETECT_CHESS = false;
const bool PUT_CIRCLES = false;
const bool FAST_METHOD = true;
const double PI = M_PI;

enum CAM_MODEL
{
    SCARA,
    MEI,
    KB,
    ATAN
};
#define MEI
#define SCARA
#define KB
#define ATAN
#define REAL_ATAN
//#define DS
//#define FOUR_CAM_SYSTEM
//#define REAL_CAM2

int px_counter = 0;

using namespace cv;
using namespace std;



int main(int argc, char** argv)
{
    string image_path = "C:/Users/Matvey/Repos/fisheye_stereo/data/stereo_img/80_stereo_shot.jpg";
    string write_path = ".";

    Mat img = imread(image_path, -1);
    Mat img0 = img(Rect(0, 0, img.cols / 2, img.rows)); //left half
    Mat img1 = img(Rect(img.cols / 2, 0, img.cols / 2, img.rows));  //right half
    Mat left = img0.clone();
    Mat right = img1.clone();
    ShowManyImages("Originals", 2, left, right);
    char key = (char)waitKey(15);

    Size origSize(1080, 1080);       //left.size();
    Size newSize(540, 540);          // determines the size of the output image
    
// Create the stereo system object
    SurroundSystem SS; 


#ifdef SCARA
    // Create the first camera object and fill its params
    ScaramuzzaModel SM3; // = SS.getCameraModel(SurroundSystem::CameraModels::SCARAMUZZA);
    SM3.setIntrinsics({ 345.1319, -0.0011, 5.7623 * pow(10, -7), -1.3985 * pow(10, -9) }, 0.022, cv::Vec2d(540, 540), cv::Matx22d(1, 0, 0, 1)); //0.0675 MRE
    SM3.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    SM3.setCamParams(origSize);
    // Create the second camera object and fill its params
    ScaramuzzaModel SM4;
    SM4.setIntrinsics({ 345.1319, -0.0011, 5.7623 * pow(10, -7), -1.3985 * pow(10, -9) }, 0.022, cv::Vec2d(540, 540), cv::Matx22d(1, 0, 0, 1));
    SM4.setExtrinsics(cv::Vec3d(1.0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795)); // 45^o
    SM4.setCamParams(origSize);

    SS.addNewCam(SM3);
    SS.addNewCam(SM4);
    SS.createStereopair(0, 1, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
#endif // SCARA
#ifdef KB
    KBModel SM5;
    SM5.setIntrinsics({ 0.000757676, -0.000325907, 0.0000403, -0.000001866 }, cv::Vec2d(540, 540), cv::Matx22d(343.536, 0, 0, 343.471));
    SM5.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    SM5.setCamParams(origSize);
    KBModel SM6;
    SM6.setIntrinsics( { 0.000757676, -0.000325907, 0.0000403, -0.000001866 }, cv::Vec2d(540, 540), cv::Matx22d(343.536, 0, 0, 343.471));
    SM6.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));
    SM6.setCamParams(origSize);

    SS.addNewCam(SM5);
    SS.addNewCam(SM6);
    SS.createStereopair(2, 3, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
#endif // KB	
    
    vector<Point> grid;                   // vectors of grid points
    vector<Point> gridDist;
    vector<Point> r_gridDist;
    int index = 2;
    int s_index = 0;
    bool recalcFlag = true;

    SS.prepareLUTs(false);
	
		
    Mat combinedRemap1(Size(newSize.width*2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
    Mat combinedRemap2(Size(newSize.width * 2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
    SS.getImage(0, SurroundSystem::RECTIFIED, left, right,  combinedRemap1);
    SS.getImage(1, SurroundSystem::RECTIFIED, left, right, combinedRemap2);

    ShowManyImages("Remapped", 2, combinedRemap1, combinedRemap2);
		
    waitKey(0);        
}
