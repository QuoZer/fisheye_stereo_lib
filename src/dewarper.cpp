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
//#define MEI
//#define SCARA
//#define KB
//#define ATAN
//#define REAL_ATAN
//#define DS
//#define FOUR_CAM_SYSTEM
#define REAL_CAM2

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
    Size newSize(1080, 1080);        // origSize * 1;            // determines the size of the output image
    
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
#ifdef DS
    DSModel SM11;
    SM11.setCamParams(origSize); // 0.57542867, -0.21840474, 
    SM11.setIntrinsics(0.570157085, -0.24504301, cv::Vec2d(0, 0), cv::Matx22d(264, 0, 0, 262));
    SM11.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.9238795, 0.3826834));
    DSModel SM12;   // 0.5648855, -0.27168128,
    SM12.setCamParams(origSize);
    SM12.setIntrinsics(0.570157085, -0.24504301, cv::Vec2d(0, 0), cv::Matx22d(264, 0, 0, 262));
    SM12.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));

    SS.addNewCam(SM11);
    SS.addNewCam(SM12);
    SS.createStereopair(10, 11, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
#endif
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
    // 2048p source
#ifdef REAL_CAM1
    Size camsize(1536, 2048 );       //imread(image_list[0], -1).size();
    Size phsize(540, 540);
    newSize = phsize;
    KBModel KB1;
    KB1.setIntrinsics({ -2.039 * pow(10, -2), 2.71859 * pow(10, -2), -1.0898 * pow(10, -2), 1.4595 * pow(10, -3) },  cv::Vec2d(725, 1119), cv::Matx22d(636, 0,0, 635));
    KB1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));  //135^o
    KB1.setCamParams(camsize);
    KBModel KB2;
    KB2.setIntrinsics({ 2.2201 * pow(10, -2), -2.6259 * pow(10, -2), 1.3064 * pow(10, -2), -2.2604 * pow(10, -3) }, cv::Vec2d(720, 989), cv::Matx22d(643, 0, 0, 643));
    KB2.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));  //-135^o
    KB2.setCamParams(camsize);
	
    ScaramuzzaModel SM1;
    SM1.setIntrinsics({ 638.649, -6.5217 * pow(10, -4), 3.0186 * pow(10, -7), -2.7871 * pow(10, -10) }, 0.022, cv::Vec2d(733, 1119), cv::Matx22d(1, 0, 0, 1));
    SM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));  //135^o
    SM1.setCamParams(camsize);
    ScaramuzzaModel SM2;
    SM2.setIntrinsics({ 648.7136, -5.4315 * pow(10, -4), 1.1746 * pow(10, -7), -1.9214 * pow(10, -10) }, 0.022, cv::Vec2d(725, 990), cv::Matx22d(1, 0, 0, 1));
    SM2.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));  //-135^o
    SM2.setCamParams(camsize);
	
    AtanModel AM1;
    AM1.setCamParams(origSize);
    AM1.setIntrinsics(cv::Vec2d(733, 1119), cv::Matx22d(1, 0, 0, 1));
    AM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    AtanModel AM2;
    AM2.setCamParams(origSize);
    AM2.setIntrinsics(cv::Vec2d(733, 1119), cv::Matx22d(1, 0, 0, 1));
    AM2.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));
	
    RealAtanModel RAM1;
    RAM1.setCamParams(origSize);
    RAM1.setIntrinsics(cv::Vec2d(733, 1119), cv::Matx22d(1, 0, 0, 1));
    RAM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    RealAtanModel RAM2;
    RAM2.setCamParams(origSize);
    RAM2.setIntrinsics(cv::Vec2d(733, 1119), cv::Matx22d(1, 0, 0, 1));
    RAM2.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));

    SS.addNewCam(KB1);
    SS.addNewCam(KB2);

    SS.addNewCam(SM1);
    SS.addNewCam(SM2);

    SS.addNewCam(AM1);
    SS.addNewCam(AM2);

    SS.addNewCam(RAM1);
    SS.addNewCam(RAM2);

    SS.createStereopair(0, 1, phsize, cv::Vec3d(0,0,0), StereoMethod::SGBM);
    SS.createStereopair(2, 3, phsize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
    SS.createStereopair(4, 5, phsize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
    SS.createStereopair(6, 7, phsize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
#endif // REAL_CAM1

#ifdef REAL_CAM2  //1080p
    Size camsize(1080, 1920);       //imread(image_list[0], -1).size();
    Size phsize(540, 540);
    newSize = phsize;
    KBModel KB1;
    KB1.setIntrinsics({ 3.778 * pow(10, -4), 4.483 * pow(10, -5), 5.799 * pow(10, -3), -2.410 * pow(10, -3) }, cv::Vec2d(498, 982), cv::Matx22d(642, 0, 0, 639));
    KB1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));  //135^o
    KB1.setCamParams(camsize);
    KBModel KB2;
    KB2.setIntrinsics({ 3.804 * pow(10, -3), 1.890 * pow(10, -3), -1.198 * pow(10, -3), 1.223 * pow(10, -4) }, cv::Vec2d(495, 836), cv::Matx22d(640, 0, 0, 641));
    KB2.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));  //-135^o
    KB2.setCamParams(camsize);

    ScaramuzzaModel SM1;
    SM1.setIntrinsics({ 647.641, -6.461 * pow(10, -4), 3.307 * pow(10, -7), -3.019 * pow(10, -10) }, 0.022, cv::Vec2d(500, 986), cv::Matx22d(1, 0, 0, 1));
    SM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));  //135^o
    SM1.setCamParams(camsize);
    ScaramuzzaModel SM2;
    SM2.setIntrinsics({ 648.7136, -5.4315 * pow(10, -4), 1.1746 * pow(10, -7), -1.9214 * pow(10, -10) }, 0.022, cv::Vec2d(500, 920), cv::Matx22d(1, 0, 0, 1));
    //SM2.setIntrinsics({ 640.3594, -5.5485 * pow(10, -4), 1.2147 * pow(10, -7), -1.8215 * pow(10, -10) }, 0.022, cv::Vec2d(500, 986), cv::Matx22d(1, 0, 0, 1));
    SM2.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));  //-135^o
    SM2.setCamParams(camsize);

    MeiModel MM1;
    MM1.setIntrinsics(2.404, 4.76583 * pow(10, -4), 2.9705 * pow(10, -4), -0.11506, 3.0141, cv::Vec2d( 488,980), cv::Matx22d(2185, 0, 0, 2176));
    MM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    MM1.setCamParams(camsize);
    MeiModel MM2;
    //MM2.setIntrinsics(2.404, 4.76583 * pow(10, -4), 2.9705 * pow(10, -4), -0.11506, 3.0141, cv::Vec2d(488, 980), cv::Matx22d(2185, 0, 0, 2176));
    MM2.setIntrinsics(1.678, 4.5441 * pow(10, -4), -5.1062 * pow(10, -3), -0.1254, 0.2757, cv::Vec2d( 495, 848), cv::Matx22d(1721, 0, 0, 1722));
    MM2.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));
    MM2.setCamParams(camsize);
	
    DSModel DSM1;
    DSM1.setCamParams(camsize); // 0.57542867, -0.21840474, 
    DSM1.setIntrinsics(-0.0706193, 0.55671, cv::Vec2d(498, 980), cv::Matx22d(580, 0, 0, 580));
    DSM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    DSModel DSM2;   // 0.5648855, -0.27168128,
    DSM2.setCamParams(camsize);
    DSM2.setIntrinsics(-0.03815, 0.624726, cv::Vec2d(486, 849), cv::Matx22d(604, 0, 0, 601));
    DSM2.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));


    AtanModel AM1;
    AM1.setCamParams(camsize);
    AM1.setIntrinsics(cv::Vec2d(733, 1119), cv::Matx22d(1, 0, 0, 1));
    AM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    AtanModel AM2;
    AM2.setCamParams(camsize);
    AM2.setIntrinsics(cv::Vec2d(733, 1119), cv::Matx22d(1, 0, 0, 1));
    AM2.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));

    RealAtanModel RAM1;
    RAM1.setCamParams(camsize);
    RAM1.setIntrinsics(cv::Vec2d(733, 1119), cv::Matx22d(1, 0, 0, 1));
    RAM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    RealAtanModel RAM2;
    RAM2.setCamParams(camsize);
    RAM2.setIntrinsics(cv::Vec2d(733, 1119), cv::Matx22d(1, 0, 0, 1));
    RAM2.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));

    SS.addNewCam(KB1);
    SS.addNewCam(KB2);

    SS.addNewCam(SM1);
    SS.addNewCam(SM2);

    SS.addNewCam(MM1);
    SS.addNewCam(MM2);
	
    SS.addNewCam(DSM1);
    SS.addNewCam(DSM2);
	
    SS.addNewCam(AM1);
    SS.addNewCam(AM2);

    SS.addNewCam(RAM1);
    SS.addNewCam(RAM2);
	
    SS.createStereopair(0, 1, phsize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
    SS.createStereopair(2, 3, phsize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
    SS.createStereopair(4, 5, phsize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
    SS.createStereopair(6, 7, phsize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
    SS.createStereopair(8, 9, phsize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
    SS.createStereopair(10, 11, phsize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
#endif // REAL_CAM2
// Create a stereosystem out of the previously created cameras (and target resolution). View direction set automatically 
    //front.setDirection()
    //if (SS.loadLUTs() == 0)
    //    {SS.prepareLUTs(true); }
	
     cv::VideoCapture camera0(cv::CAP_DSHOW);
     camera0.set(cv::CAP_PROP_FOURCC ,VideoWriter::fourcc('M', 'J', 'P', 'G'));
     camera0.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
     camera0.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
     camera0.set(cv::CAP_PROP_FPS, 15);
	
     cv::VideoCapture camera1(1+cv::CAP_DSHOW);
     camera1.set(cv::CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
     camera1.set(cv::CAP_PROP_FRAME_WIDTH, 1920);
     camera1.set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
     camera1.set(cv::CAP_PROP_FPS, 15);
	
     if (!camera0.isOpened() || !camera1.isOpened()) {
         cerr << "ERROR! Unable to open camera\n";
         return -1;
     } 

    string path = "D:/Work/Coding/Repos/fisheye_stereo/data/1080p[REAL]"; 
    string read_path = path + "/calib_imgs/";
    string write_path = path + "/3_Compar0.072m/";
	
    
    vector<Point> grid;                   // vectors of grid points
    vector<Point> gridDist;
    vector<Point> r_gridDist;
    int index = 0;
    bool recalcFlag = true;
    SS.prepareLUTs(false);
	
	Mat img0, img1;
    while(true)         //  iterate through images       
    {
		camera0.read(img1);
        camera1.read(img0);
		
        //Mat img0 = imread(basepath + "Left/image" + std::to_string(index) + ".png", -1);
        //Mat img1 = imread(basepath + "Right/image" + std::to_string(index) + ".png", -1);
		
        Mat left = img0.clone();
        Mat right = img1.clone();     
		ShowManyImages("Origs", 2, left, right);

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
        SS.getImage(0, SurroundSystem::RECTIFIED, left, right,  combinedRemap);
		
        cv::imshow("disparity", combinedRemap);

        char key = (char)waitKey(15);
        switch (key){
        case 'r':
            index += 0;
            cout << "Reload" << endl;
            break;
        case 'o':
            cout << "Saving originals" << endl;
            imwrite(write_path + "l_image" + std::to_string(index) + ".png", left);
            imwrite(write_path + "r_image" + std::to_string(index) + ".png", right);
            //index++;
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
            saveWithAllModels(write_path, SS, left, right,  index);
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
        
        //if (index == n_img || index < 0) index = 0;
    }
}
