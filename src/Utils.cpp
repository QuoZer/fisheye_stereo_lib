#include "Utils.h"


using cv::Mat; 
using cv::Size;
using cv::Scalar;
using cv::Vec3b;

using std::string;
using std::vector;
using std::cout;
using std::endl;


void cropUglyScreenshots(const vector<string>& list)
{
    int n_img = (int)list.size();
    for (int i = 0; i < n_img; ++i)
    {
        Mat img = cv::imread(list[i], -1);
        Mat beauty = img(cv::Rect(0, 0, 500, 500)).clone();
        string path = "t" + std::to_string(i) + "_crop.jpg";
        cout << path << endl;
        imwrite(path, beauty);
    }
}

// beta = brightness, alpha = contrast
void changeContrastAndBrightness(cv::Mat& image, double alpha = 1, int beta = 0)
{
    for (int y = 0; y < image.rows; y++) {
        for (int x = 0; x < image.cols; x++) {
            for (int c = 0; c < image.channels(); c++) {
                image.at<cv::Vec3b>(y, x)[c] =
                    cv::saturate_cast<uchar>(alpha * image.at<cv::Vec3b>(y, x)[c] + beta);
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
    cv::Mat DispImage = cv::Mat::zeros(cv::Size(100 + size * w, 60 + size * h), CV_8UC3);
    // Used to get the arguments passed
    va_list args;
    va_start(args, nArgs);
    // Loop for nArgs number of arguments
    for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {
        // Get the Pointer to the IplImage
        cv::Mat img = va_arg(args, cv::Mat);

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
        cv::Rect ROI(m, n, (int)(x / scale), (int)(y / scale));
        cv::Mat temp; resize(img, temp, cv::Size(ROI.width, ROI.height));
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

std::map<int, string> types = { {0, "MEI"}, {1, "SCARA"}, {2, "KB"}, {3, "ATAN"}, {4, "REAL_ATAN"}};

void saveWithAllModels(SurroundSystem& SS, cv::Mat& left, cv::Mat& right, int index)
{
    Size newSize(540, 540);
    Mat combinedRemap(Size(newSize.width * 2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
    for (int i = 0; i < SS.getNumOfSP(); i++)
    {
        SS.getImage(i, SurroundSystem::RECTIFIED, left, right, combinedRemap);
        Mat left_rem = combinedRemap(cv::Rect(0, 0, newSize.width, newSize.height)).clone();
        Mat right_rem = combinedRemap(cv::Rect(newSize.width, 0, newSize.width, newSize.height)).clone();     
        string folder = "D:/Work/Coding/Repos/fisheye_stereo/data/3_Compar0.1m/" + types[i] + "/";

        string l_name = folder + "l_img_" + types[i] + std::to_string(index) + ".png";      // exmpl: l_img_ATAN3.png
        string r_name = folder + "r_img_" + types[i] + std::to_string(index) + ".png";
        cout << "Saving images..." << types[i] << index << endl;
        imwrite(l_name, left_rem);
        imwrite(r_name, right_rem);
    }

}

void savePano(SurroundSystem& SS, cv::Mat& combinedRemap, int type_i, int sp_index)
{
    int i = sp_index;
	
    Size newSize(540, 540);

    Mat left_rem = combinedRemap(cv::Rect(0, 0, newSize.width, newSize.height)).clone();
    Mat right_rem = combinedRemap(cv::Rect(newSize.width, 0, newSize.width, newSize.height)).clone();
    string folder = "D:/Work/Coding/Repos/fisheye_stereo/data/1_4System0.1m/" + types[type_i] + "/";

    string l_name = folder + std::to_string(i) + "_l_img" + ".png"; // exmpl: 0_l_img.png       SP number is left camera index
    string r_name = folder + std::to_string(i) + "_r_img" + ".png";
    cout << "Saving images..." << types[type_i] << " | " << i << "/" << SS.getNumOfSP() << endl;
    imwrite(l_name, left_rem);
    imwrite(r_name, right_rem);
    
	
}

bool readStringList(const string& filename, vector<string>& l)
{
    l.resize(0);
    cv::FileStorage fs(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
        return false;
    cv::FileNode n = fs.getFirstTopLevelNode();
    if (n.type() != cv::FileNode::SEQ)
        return false;
    cv::FileNodeIterator it = n.begin(), it_end = n.end();
    for (; it != it_end; ++it)
        l.push_back((string)*it);
    return true;
}