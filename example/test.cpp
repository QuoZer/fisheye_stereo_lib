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

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    // test image path 
    string image_path = "C:/Users/Matvey/Repos/fisheye_stereo/data/stereo_img/80_stereo_shot.jpg";
    string write_path = ".";
    // read the image and separate into two 
    Mat img = imread(image_path, -1);
    Mat img0 = img(Rect(0, 0, img.cols / 2, img.rows)); //left half
    Mat img1 = img(Rect(img.cols / 2, 0, img.cols / 2, img.rows));  //right half
    Mat left = img0.clone();
    Mat right = img1.clone();
    // display input
    ShowManyImages("Originals", 2, left, right);
    waitKey(15);

    Size origSize(1080, 1080);       //left.size();
    Size newSize(540, 540);          // determines the size of the output image
    
/*  1. Create the stereo system object    */ 
    SurroundSystem SS("svs");

/*  2. Describe your system. */

// SCARAMUZZA
    // Create the first camera object and fill its params
    ScaramuzzaModel SM1; // = SS.getCameraModel(SurroundSystem::CameraModels::SCARAMUZZA);
    SM1.setIntrinsics({ 345.1319, -0.0011, 5.7623 * pow(10, -7), -1.3985 * pow(10, -9) }, 0.022, cv::Vec2d(540, 540), cv::Matx22d(1, 0, 0, 1)); //0.0675 MRE
    SM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    SM1.setCamParams(origSize);
    // Create the second camera object and fill its params
    ScaramuzzaModel SM2;
    SM2.setIntrinsics({ 345.1319, -0.0011, 5.7623 * pow(10, -7), -1.3985 * pow(10, -9) }, 0.022, cv::Vec2d(540, 540), cv::Matx22d(1, 0, 0, 1));
    SM2.setExtrinsics(cv::Vec3d(1.0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795)); // 45^o
    SM2.setCamParams(origSize);

    SS.addNewCam(SM1);
    SS.addNewCam(SM2);
    SS.createStereopair(0, 1, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);

// KANNALA-BRANDT     
    KBModel KBM1;
    KBM1.setIntrinsics({ 0.000757676, -0.000325907, 0.0000403, -0.000001866 }, cv::Vec2d(540, 540), cv::Matx22d(343.536, 0, 0, 343.471));
    KBM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    KBM1.setCamParams(origSize);
    KBModel KBM2;
    KBM2.setIntrinsics( { 0.000757676, -0.000325907, 0.0000403, -0.000001866 }, cv::Vec2d(540, 540), cv::Matx22d(343.536, 0, 0, 343.471));
    KBM2.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));
    KBM2.setCamParams(origSize);

    SS.addNewCam(KBM1);
    SS.addNewCam(KBM2);
    SS.createStereopair(2, 3, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);

/*  3. Compute look-up tables  */
    //SS.prepareLUTs(true);

/*  3.1 OR load already filled ones  */
    SS.loadLUTs("svs");
	
/*  4. Get images  */
    Mat combinedRemap1(Size(newSize.width*2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
    Mat combinedRemap2(Size(newSize.width * 2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
    SS.getImage(0, SurroundSystem::RECTIFIED, left, right,  combinedRemap1);
    SS.getImage(1, SurroundSystem::RECTIFIED, left, right, combinedRemap2);

/*  5. See the results  */
    ShowManyImages("Remapped", 2, combinedRemap1, combinedRemap2);
    waitKey(0);        
}
