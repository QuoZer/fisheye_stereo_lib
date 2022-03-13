#include "opencv2/ccalib/omnidir.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <time.h>
#include <cstdarg>
#include "SurroundSystem.hpp"


const bool DETECT_CHESS = false;
const bool PUT_CIRCLES = false;
const bool FAST_METHOD = true;
const double PI = M_PI;

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
    Size newSize(540, 540);        // origSize * 1;            // determines the size of the output image
    
// Create the stereo system object
    SurroundSystem SS; 
// Create the first camera object and fill its params
    ScaramuzzaModel SM1; // = SS.getCameraModel(SurroundSystem::CameraModels::SCARAMUZZA);
    SM1.setIntrinsics({ 350.8434, -0.0015, 2.1981 * pow(10, -6), -3.154 * pow(10, -9) }, cv::Vec2d(0, 0), cv::Matx22d(1, 0, 0, 1), 0.022);
    SM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0, 0, 0, 1));
    SM1.setCamParams(origSize);
// Create the second camera object and fill its params
    ScaramuzzaModel SM2;
    SM2.setIntrinsics({ 350.8434, -0.0015, 2.1981 * pow(10, -6), -3.154 * pow(10, -9) }, cv::Vec2d(0, 0), cv::Matx22d(1, 0, 0, 1), 0.022);
    SM2.setExtrinsics(cv::Vec3d(1.0, 0, 0), cv::Vec4d(0, 0, 0.7071068, 0.7071068)); // 90^o
    SM2.setCamParams(origSize);
// Add these cams to the stereosystem
    SS.addNewCam(SM1);
    SS.addNewCam(SM2);
// Create a stereosystem out of the previously created cameras (and target resolution). View direction set automatically 
    int SPindex = SS.createStereopair(0, 1, newSize, cv::Vec3d(0,0,0), StereoMethod::SGBM);
    //front.setDirection()
    SS.prepareLUTs(); 
    

    vector<Point> grid;                   // vectors of grid points
    vector<Point> gridDist;
    vector<Point> r_gridDist;
    //ScaramuzzaModel SM1;
    //SM1.setIntrinsics({ 350.8434, -0.0015, 2.1981 * pow(10, -6), -3.154 * pow(10, -9) }, cv::Vec2d(0, 0), cv::Matx22d(1, 0, 0, 1), 0.022);
    //SM1.setExtrinsics(cv::Vec3d(0, 0, 0), cv::Vec4d(0,0,0,1));
    //FisheyeDewarper dewarper(&SM1);
    //dewarper.setSize(origSize, newSize, 90);
    //dewarper.setRpy(0, 0, 0);
    //
    //ScaramuzzaModel SM2;
    //SM2.setIntrinsics({ 350.8434, -0.0015, 2.1981 * pow(10, -6), -3.154 * pow(10, -9) }, cv::Vec2d(0, 0), cv::Matx22d(1, 0, 0, 1), 0.022);
    //SM2.setExtrinsics(cv::Vec3d(1.0, 0, 0), cv::Vec4d(0, 0, 0, 1));
    //FisheyeDewarper r_dewarper(&SM2);
    //r_dewarper.setSize(origSize, newSize, 90);
    //r_dewarper.setRpy(0, 0, 0);


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
        case 'z':
            exit(0);
        default: 
            index +=0;
            break;
        }
        
        if (index == n_img - 1 || index < 0) index = 0;
    }
}
