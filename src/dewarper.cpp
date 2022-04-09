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

// args: -w=9 -h=6 -pt=chessboard ..\..\data\stereo_img\two_dome_images\stereo_list.xml

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
#define ATAN

int px_counter = 0;

using namespace cv;
using namespace std;


static void cropUglyScreenshots(const vector<string>& list)
{
    int n_img = (int)list.size();
    for (int i = 0; i < n_img; ++i)
    {
        Mat img = imread(list[i], -1);
        Mat beauty = img(Rect(0, 0, 500, 500)).clone();
        string path = "t" + to_string(i) + "_crop.jpg";
        cout << path << endl;
        imwrite(path, beauty);
    }
}

// beta = brightness, alpha = contrast
void changeContrastAndBrightness(Mat& image, double alpha = 1, int beta = 0)
{
    for (int y = 0; y < image.rows; y++) {
        for (int x = 0; x < image.cols; x++) {
            for (int c = 0; c < image.channels(); c++) {
                image.at<Vec3b>(y, x)[c] =
                    saturate_cast<uchar>(alpha * image.at<Vec3b>(y, x)[c] + beta);
            }
        }
    }
}

void ShowManyImages(string title, int nArgs, ...) {
    int size;
    int i;
    int m, n;
    int x, y;

    // w - Maximum number of images in a row
    // h - Maximum number of images in a column
    int w, h;

    // scale - How much we have to resize the image
    float scale;
    int max;

    // If the number of arguments is lesser than 0 or greater than 12
    // return without displaying
    if (nArgs <= 0) {
        printf("Number of arguments too small....\n");
        return;
    }
    else if (nArgs > 14) {
        printf("Number of arguments too large, can only handle maximally 12 images at a time ...\n");
        return;
    }
    // Determine the size of the image,
    // and the number of rows/cols
    // from number of arguments
    else if (nArgs == 1) {
        w = h = 1;
        size = 1080;
    }
    else if (nArgs == 2) {
        w = 2; h = 1;
        size = 540;
    }
    else if (nArgs == 3 || nArgs == 4) {
        w = 2; h = 2;
        size = 540;
    }
    else if (nArgs == 5 || nArgs == 6) {
        w = 3; h = 2;
        size = 200;
    }
    else if (nArgs == 7 || nArgs == 8) {
        w = 4; h = 2;
        size = 200;
    }
    else {
        w = 4; h = 3;
        size = 150;
    }
    // Create a new 3 channel image
    Mat DispImage = Mat::zeros(Size(100 + size * w, 60 + size * h), CV_8UC3);
    // Used to get the arguments passed
    va_list args;
    va_start(args, nArgs);
    // Loop for nArgs number of arguments
    for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {
        // Get the Pointer to the IplImage
        Mat img = va_arg(args, Mat);

        // Check whether it is NULL or not
        // If it is NULL, release the image, and return
        if (img.empty()) {
            printf("Invalid arguments");
            return;
        }
        // Find the width and height of the image
        x = img.cols;
        y = img.rows;
        // Find whether height or width is greater in order to resize the image
        max = (x > y) ? x : y;

        // Find the scaling factor to resize the image
        scale = (float)((float)max / size);

        // Used to Align the images
        if (i % w == 0 && m != 20) {
            m = 20;
            n += 20 + size;
        }
        // Set the image ROI to display the current image
        // Resize the input image and copy the it to the Single Big Image
        Rect ROI(m, n, (int)(x / scale), (int)(y / scale));
        Mat temp; resize(img, temp, Size(ROI.width, ROI.height));
        temp.copyTo(DispImage(ROI));
    }

    // Create a new window, and show the Single Big Image
    //namedWindow(title, 1);
    imshow(title, DispImage);
    //waitKey();
    //waitKey(15);
    // End the number of arguments
    va_end(args);
}

map<int, string> types = { {0, "MEI"}, {1, "SCARA"}, {2, "KB"}, {3, "ATAN"} };

void saveAllImages(SurroundSystem& SS, cv::Mat& left, cv::Mat& right, int index )
{
    Size newSize(1080, 1080);
    Mat combinedRemap(Size(newSize.width * 2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
    for (int i = 0; i < SS.getNumOfSP(); i++)
    {
        SS.getImage(i, SurroundSystem::RECTIFIED, left, right, combinedRemap);
        Mat left_rem = combinedRemap(Rect(0, 0, 1080, 1080)).clone();
        Mat right_rem = combinedRemap(Rect(1080, 0, 1080, 1080)).clone();
        string folder = "./N_Compar0.1m/"+types[i]+"/";
		
        string l_name =  "l_img_" + types[i] + to_string(index) + ".png";
        string r_name =  "r_img_" + types[i] + to_string(index) + ".png";
        cout << "Saving images..." << types[i] << index << endl;
        imwrite(l_name, left_rem);
        imwrite(r_name, right_rem);
    }

}

static bool readStringList(const string& filename, vector<string>& l)
{
    l.resize(0);
    FileStorage fs(filename, FileStorage::READ);
    if (!fs.isOpened())
        return false;
    FileNode n = fs.getFirstTopLevelNode();
    if (n.type() != FileNode::SEQ)
        return false;
    FileNodeIterator it = n.begin(), it_end = n.end();
    for (; it != it_end; ++it)
        l.push_back((string)*it);
    return true;
}


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

    int index = 2;
    bool recalcFlag = true;

    Size origSize(1080, 1080);       //imread(image_list[0], -1).size();
    Size newSize(1080, 1080);        // origSize * 1;            // determines the size of the output image
    
// Create the stereo system object
    SurroundSystem SS; 

#ifdef MEI
    MeiModel SM1;
    SM1.setIntrinsics(1.47431, 0.000166, 0.00008, -0.208916, 0.153247, cv::Vec2d(0, 0), cv::Matx22d(849, 0, 0, 849));
    SM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    SM1.setCamParams(origSize);
    MeiModel SM2;
    SM2.setIntrinsics(1.47431, 0.000166, 0.00008, -0.208916, 0.153247, cv::Vec2d(0, 0), cv::Matx22d(849, 0, 0, 849));
    SM2.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, -0.3826834, 0.9238795));
    SM2.setCamParams(origSize);
    
    SS.addNewCam(SM1);
    SS.addNewCam(SM2);
    SS.createStereopair(0, 1, newSize, cv::Vec3d(0,0,0), StereoMethod::SGBM);
#endif // MEI
#ifdef SCARA
    // Create the first camera object and fill its params
    ScaramuzzaModel SM3; // = SS.getCameraModel(SurroundSystem::CameraModels::SCARAMUZZA);
    SM3.setIntrinsics({ 344.3100, -0.0010, 4.0682 * pow(10, -7), -1.2138 * pow(10, -9) }, 0.022, cv::Vec2d(0, 0), cv::Matx22d(1, 0, 0, 1));
    SM3.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0.3826834, 0.9238795));
    SM3.setCamParams(origSize);
    // Create the second camera object and fill its params
    ScaramuzzaModel SM4;
    SM4.setIntrinsics({ 344.3100, -0.0010, 4.0682 * pow(10, -7), -1.2138 * pow(10, -9) }, 0.022, cv::Vec2d(0, 0), cv::Matx22d(1, 0, 0, 1));
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
    SS.createStereopair(0, 1, newSize, cv::Vec3d(0, 0, 0), StereoMethod::SGBM);
#endif // ATAN


 // Add these cams to the stereosystem

// Create a stereosystem out of the previously created cameras (and target resolution). View direction set automatically 
    SS.prepareLUTs(); 
    

    vector<Point> grid;                   // vectors of grid points
    vector<Point> gridDist;
    vector<Point> r_gridDist;

    while(true)         //  iterate through images       
    {
        Mat img = imread(image_list[index], -1);
        Mat right = img(Rect(0, 0, 1080, 1080)).clone();
        Mat left = img(Rect(1080, 0, 1080, 1080)).clone();

        if (recalcFlag){
                                                       
            //dewarper.fillMaps();                      // fill new maps with current parameters. 
            //r_dewarper.fillMaps();
            cout << "Maps ready" << endl;

            recalcFlag = false;
        }

        //Mat leftImageRemapped(newSize, CV_8UC3, Scalar(0, 0, 0));
        //Mat rightImageRemapped(newSize, CV_8UC3, Scalar(0, 0, 0));
        Mat combinedRemap(Size(newSize.width*2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
        SS.getImage(0, SurroundSystem::RECTIFIED, left, right, combinedRemap);

        //bool textPut = true;
        //// draw grid
        //for each (Point center in gridDist)
        //{
        //    if (!textPut) {
        //        Point textOrigin = center - Point(20,20);
        //        textPut = true;
        //    }
        //    circle(left, center, 4, Scalar(115, 25, 10), 3);
        //}
        //for each (Point center in r_gridDist)
        //{
        //    if (!textPut) {
        //        Point textOrigin = center - Point(20, 20);
        //        textPut = true;
        //    }
        //    circle(right, center, 4, Scalar(115, 25, 10), 3);
        //}

        //std::vector<cv::Point> hull;
        //convexHull(gridDist, hull);
        //cout << "Contour area is" << contourArea(hull) << std::endl;

        // Converting images to grayscale
        //cv::cvtColor(leftImageRemapped, leftImageRemapped, cv::COLOR_BGR2GRAY);
        //cv::cvtColor(rightImageRemapped, rightImageRemapped, cv::COLOR_BGR2GRAY);
        //ShowManyImages("Images", 4, right, left,leftImageRemapped, rightImageRemapped  );
        //imwrite("rightImageRemapped.png", rightImageRemapped);
        // Displaying the disparity map
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
            saveAllImages(SS, left, right, index);
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
