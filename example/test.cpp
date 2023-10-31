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
    string left_path = "C:/Users/Matvey/Repos/fisheye_stereo/data/ELTE/90deg/Dev1_Image_w1920_h1200_fn6.bmp";
    string right_path = "C:/Users/Matvey/Repos/fisheye_stereo/data/ELTE/90deg/Dev0_Image_w1920_h1200_fn6.bmp";
    string write_path = ".";
    // read the image and separate into two 
    Mat left = imread(left_path, -1);
    Mat right = imread(right_path, -1);
    // display input
    ShowManyImages("Originals", 2, left, right);
    waitKey(15);

    Size origSize(1920, 1200);       //left.size();
    Size newSize(540, 540);          // determines the size of the output image
    
/*  1. Create the stereo system object    */ 
    SurroundSystem SS("elte");
/*  2. Describe your system. */
    SS.readSystemParams("C:/Users/Matvey/Repos/fisheye_stereo/fy_lib_source/elte_params.xml");

/*  3. Compute look-up tables  */
    SS.prepareLUTs(false);

/*  3.1 OR load already filled ones (WIP) */
    //SS.loadLUTs("svs");
	
/*  4. Get images  */
    Mat combinedRemap1(Size(newSize.width*2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
    Mat disparity1(newSize, CV_32F, Scalar(0, 0, 0));
    
    SS.getImage(0, SurroundSystem::DISPARITY, left, right, disparity1);
    SS.getImage(0, SurroundSystem::RECTIFIED, left, right, combinedRemap1);
    

    // Save combinedRemap
    //imwrite("./combinedRemap1.png", combinedRemap1);

/*  5. See the results  */
    imshow("Remapped", combinedRemap1);
    //ShowManyImages("Remapped", 2, combinedRemap1, combinedRemap2);
    waitKey(0);        
}
