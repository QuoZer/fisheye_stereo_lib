#include "opencv2/ccalib/omnidir.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <time.h>
#include <map>
#include <cstdarg>
#include "SurroundSystem.hpp"
#include "Utils.h"

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
//#define FOUR_CAM_SYSTEM

int px_counter = 0;

using namespace cv;
using namespace std;



int main(int argc, char** argv)
{
    cv::CommandLineParser parser(argc, argv,
        "{@input||input file - xml file with a list of the images, created with cpp-example-imagelist_creator tool}"
        "{help||show help}"
    );
    parser.about("This is a sample for fisheye camera undistortion. Example command line:\n"
        "    dewrapper imagelist.xml \n");
    if (parser.has("help"))
    {
        parser.printMessage();
        return 0;
    }
    const string inputFilename = parser.get<string>(0);
    if (!parser.check())
    {
        parser.printErrors();
        return -1;
    }

    // get image name list
    vector<string> image_list, detec_list;
    if (!readStringList(inputFilename, image_list))
    {
        cout << "Can not read imagelist" << endl;
        return -1;
    }

    int n_img = (int)image_list.size();
    cout << "Images: " << n_img << endl;



    Size origSize(1080, 1080);       //imread(image_list[0], -1).size();
    Size newSize(540, 540);        // origSize * 1;            // determines the size of the output image
    
// Create the stereo system object
    SurroundSystem SS; 

#ifdef MEI
    MeiModel SM1;
    SM1.setIntrinsics(1.47431, 0.000166, 0.00008, -0.208916, 0.153247, cv::Vec2d(0, 0), cv::Matx22d(849, 0, 0, 849));
    SM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.9238795, 0.3826834));
    SM1.setCamParams(origSize);
    MeiModel SM2;
    SM2.setIntrinsics(1.47431, 0.000166, 0.00008, -0.208916, 0.153247, cv::Vec2d(0, 0), cv::Matx22d(849, 0, 0, 849));
    SM2.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    SM2.setCamParams(origSize);
    
    SS.addNewCam(SM1);
    SS.addNewCam(SM2);
    SS.createStereopair(0, 1, newSize, cv::Vec3d(0,0,0), StereoMethod::SGBM);
#endif // MEI
#ifdef SCARA
    // Create the first camera object and fill its params
    ScaramuzzaModel SM3; // = SS.getCameraModel(SurroundSystem::CameraModels::SCARAMUZZA);
    SM3.setIntrinsics({ 345.1319, -0.0011, 5.7623 * pow(10, -7), -1.3985 * pow(10, -9) }, 0.022, cv::Vec2d(0, 0), cv::Matx22d(1, 0, 0, 1));
    SM3.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    SM3.setCamParams(origSize);
    // Create the second camera object and fill its params
    ScaramuzzaModel SM4;
    SM4.setIntrinsics({ 345.1319, -0.0011, 5.7623 * pow(10, -7), -1.3985 * pow(10, -9) }, 0.022, cv::Vec2d(0, 0), cv::Matx22d(1, 0, 0, 1));
    SM4.setExtrinsics(cv::Vec3d(1.0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795)); // 45^o
    SM4.setCamParams(origSize);

    SS.addNewCam(SM3);
    SS.addNewCam(SM4);
    SS.createStereopair(2, 3, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
#endif // SCARA
#ifdef KB
    KBModel SM5;
    SM5.setIntrinsics({ 0.000757676, -0.000325907, 0.0000403, -0.000001866 }, cv::Vec2d(0, 0), cv::Matx22d(343.536, 0, 0, 343.471));
    SM5.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    SM5.setCamParams(origSize);
    KBModel SM6;
    SM6.setIntrinsics( { 0.000757676, -0.000325907, 0.0000403, -0.000001866 }, cv::Vec2d(0, 0), cv::Matx22d(343.536, 0, 0, 343.471));
    SM6.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));
    SM6.setCamParams(origSize);

    SS.addNewCam(SM5);
    SS.addNewCam(SM6);
    SS.createStereopair(4, 5, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
#endif // KB
#ifdef ATAN
    AtanModel SM7;
    SM7.setCamParams(origSize);
    SM7.setIntrinsics(cv::Vec2d(0, 0), cv::Matx22d(1, 0, 0, 1));
    SM7.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    AtanModel SM8;
    SM8.setCamParams(origSize);
    SM8.setIntrinsics( cv::Vec2d(0, 0), cv::Matx22d(1, 0, 0, 1));
    SM8.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));
	
    SS.addNewCam(SM7);
    SS.addNewCam(SM8);
    SS.createStereopair(6, 7, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
#endif // ATAN
#ifdef REAL_ATAN
    RealAtanModel SM9;
    SM9.setCamParams(origSize);
    SM9.setIntrinsics(cv::Vec2d(0, 0), cv::Matx22d(1, 0, 0, 1));
    SM9.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.9238795, 0.3826834 ));
    RealAtanModel SM10;
    SM10.setCamParams(origSize);
    SM10.setIntrinsics(cv::Vec2d(0, 0), cv::Matx22d(1, 0, 0, 1));
    SM10.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));

    SS.addNewCam(SM9);
    SS.addNewCam(SM10);
    SS.createStereopair(8, 9, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
#endif // ATAN
#ifdef FOUR_CAM_SYSTEM
    KBModel SM0;
    SM0.setIntrinsics({ 0.000757676, -0.000325907, 0.0000403, -0.000001866 }, cv::Vec2d(0, 0), cv::Matx22d(343.536, 0, 0, 343.471));
    SM0.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));  //45^o
    SM0.setCamParams(origSize);
    KBModel SM1;
    SM1.setIntrinsics({ 0.000757676, -0.000325907, 0.0000403, -0.000001866 }, cv::Vec2d(0, 0), cv::Matx22d(343.536, 0, 0, 343.471));
    SM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));  //-45^o
    SM1.setCamParams(origSize);
    KBModel SM2;
    SM2.setIntrinsics({ 0.000757676, -0.000325907, 0.0000403, -0.000001866 }, cv::Vec2d(0, 0), cv::Matx22d(343.536, 0, 0, 343.471));
    SM2.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.9238795, 0.3826834));   //135^o
    SM2.setCamParams(origSize);
    KBModel SM3;
    SM3.setIntrinsics({ 0.000757676, -0.000325907, 0.0000403, -0.000001866 }, cv::Vec2d(0, 0), cv::Matx22d(343.536, 0, 0, 343.471));
    SM3.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.9238795, 0.3826834));  //-135^o
    SM3.setCamParams(origSize);

    SS.addNewCam(SM0);
    SS.addNewCam(SM1);
	SS.addNewCam(SM2);
    SS.addNewCam(SM3);
    SS.createStereopair(0, 1, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
    SS.createStereopair(1, 3, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
    SS.createStereopair(2, 0, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
    SS.createStereopair(3, 2, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
	
#endif

// Create a stereosystem out of the previously created cameras (and target resolution). View direction set automatically 
    SS.prepareLUTs(); 

    string path = "D:/Work/Coding/Repos/fisheye_stereo/data/1_Compar0.1m/"; //TODO: exctract from argument
    
    vector<Point> grid;                   // vectors of grid points
    vector<Point> gridDist;
    vector<Point> r_gridDist;
    int index = 0;
    bool recalcFlag = true;
    while(true)         //  iterate through images       
    {
        Mat img = imread(image_list[index], -1);
        Mat right = img(Rect(0, 0, 1080, 1080)).clone();                // FIXME: WRONG L/R
        Mat left = img(Rect(1080, 0, 1080, 1080)).clone();

        if (recalcFlag){
            cout << "Maps ready" << endl;

            recalcFlag = false;
        }

#ifdef FOUR_CAM_SYSTEM
        Mat img_front = imread(image_list[0], -1);
		Mat img_back  = imread(image_list[1], -1);
        Mat left_front  = img_front(Rect(0, 0, 1080, 1080)).clone();                // TODO: remake
        Mat right_front = img_front(Rect(1080, 0, 1080, 1080)).clone();
		Mat left_back   = img_back( Rect(0, 0, 1080, 1080)).clone();
		Mat right_back  = img_back( Rect(1080, 0, 1080, 1080)).clone();
		
        Mat combinedRemap1(Size(newSize.width * 2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
        SS.getImage(0, SurroundSystem::RECTIFIED, left_front, right_front, combinedRemap1);
        Mat combinedRemap2(Size(newSize.width * 2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
        SS.getImage(1, SurroundSystem::RECTIFIED, right_front, left_back, combinedRemap2);
        Mat combinedRemap3(Size(newSize.width * 2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
        SS.getImage(2, SurroundSystem::RECTIFIED, right_back, left_front, combinedRemap3);
        Mat combinedRemap4(Size(newSize.width * 2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
        SS.getImage(3, SurroundSystem::RECTIFIED, left_back, right_back, combinedRemap4);

        ShowManyImages("Images", 4, combinedRemap1, combinedRemap2, combinedRemap3, combinedRemap4);

#endif // FOUR_CAM_SYSTEM

		
        Mat combinedRemap(Size(newSize.width*2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
        SS.getImage(0, SurroundSystem::RECTIFIED, right,left,  combinedRemap);

		
        cv::imshow("disparity", combinedRemap);

        char key = (char)waitKey(0);
        switch (key){
        case 'r':
            index += 0;
            cout << "Reload" << endl;
            break;
        case 'q':
            index--;
            cout << "Prev image" << endl;
            break;
        case 'e':
            index++;
            cout << "Next image" << endl;
            break;
        case 'd':
            cout << "calculating depth" << endl;
            // depthSwitcher = true;
            break;
        case 's': {
            saveWithAllModels(path, SS, right,left,  index);
            //savePano(SS, combinedRemap1, 2, 0);
            //savePano(SS, combinedRemap2, 2, 1);
            //savePano(SS, combinedRemap3, 2, 2);
            //savePano(SS, combinedRemap4, 2, 3);
            break; }
        case 'z':
            exit(0);
        default: 
            index +=0;
            break;
        }
        
        if (index == n_img || index < 0) index = 0;
    }
}
