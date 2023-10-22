//#include "opencv2/ccalib/omnidir.hpp"
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
    string image_path = "C:/Users/Matvey/Repos/fisheye_stereo/data/stereo_img/38_stereo_shot.jpg";
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
    SS.readSystemParams("C:/Users/Matvey/Repos/fisheye_stereo/fy_lib_source/svs_params.xml");

/*  3. Compute look-up tables  */
    SS.prepareLUTs(true);

/*  3.1 OR load already filled ones (WIP) */
    //SS.loadLUTs("svs");
	
/*  4. Get images  */
    Mat combinedRemap1(Size(newSize.width*2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
    Mat combinedRemap2(Size(newSize.width * 2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
    Mat disparity1(newSize, CV_32F, Scalar(0, 0, 0));
    Mat disparity2(newSize, CV_32F, Scalar(0, 0, 0));
    SS.getImage(0, SurroundSystem::DISPARITY, left, right, disparity1);
    SS.getImage(1, SurroundSystem::DISPARITY, left, right, disparity2);
    SS.getImage(0, SurroundSystem::RECTIFIED, left, right, combinedRemap1);
    SS.getImage(1, SurroundSystem::RECTIFIED, left, right, combinedRemap2);

    // Save combinedRemap
    //imwrite("./combinedRemap1.png", combinedRemap1);

/*  5. See the results  */
    //imshow("Disparity", disparity2);
    ShowManyImages("Remapped", 2, combinedRemap1, combinedRemap2);
    waitKey(0);        
}
