//#include "opencv2/ccalib/omnidir.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/videoio.hpp>
#include <vector>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <string>
#include <time.h>
#include <map>
#include <cstdarg>
#include "SurroundSystem.hpp"
#include "Utils.hpp"

using namespace cv;
using namespace std;

int resavePLY(string inputFileName, string outputFileName)
{
    // Open the input file
	std::ifstream inputFile(inputFileName, std::ios::binary);
    if (!inputFile) {
		std::cerr << "Error: Unable to open input file: " << inputFileName << std::endl;
		return 1;
	}

	// Open the output file
	std::ofstream outputFile(outputFileName, std::ios::binary);
    if (!outputFile) {
		std::cerr << "Error: Unable to open output file: " << outputFileName << std::endl;
		return 1;
	}

	// Read data from the input file and write it to the output file
	outputFile << inputFile.rdbuf();

	// Close the files
	inputFile.close();
	outputFile.close();
}

// Function to get a number of files in a directory
int getNumFiles(string path)
{
	int num_files = 0;
	for (auto const& entry : filesystem::directory_iterator(path))
		num_files++;
	return num_files;
}


int main(int argc, char** argv)
{ 
    string base_path = "C:/Users/Matvey/Repos/fisheye_stereo/data/ELTE2/120deg/";
    string param_path = "C:/Users/Matvey/Repos/fisheye_stereo/fy_lib_source/config/elte_params_120deg.xml";

    // indices of the measurement images to be dewarped
    //std::vector<int> indicies = { 1, 6, 11, 16, 21, 27, 31, 36, 41, 45 }; // 0 deg
    //std::vector<int> indicies = { 1, 7, 12, 17, 22, 27, 32, 37, 41, 45}; // 30 deg
    //std::vector<int> indicies = { 1, 6, 11, 16, 21, 27, 31, 36, 41, 45 }; // 60 deg
    //std::vector<int> indicies = {1, 6, 11, 16, 21, 27, 31, 36, 41, 45}; // 90 deg
    std::vector<int> indicies = {4, 6, 11, 16, 21, 26, 31, 36, 41}; // 120 deg
    // count the number of calibration image pairs
    int num_distances = getNumFiles(base_path + "calibration") / 2;

    bool swap_lr = true;

    SurroundSystem SS("elte");
    SS.readSystemParams(param_path);
    SS.prepareLUTs(false);
    Size newSize(540, 540);          // determines the size of the output image

    // loop over all measurements 
    int new_index = 1;
    for (int index : indicies)
    {
        string left_path, right_path, scan_path, new_scan_path;
        if (swap_lr)
        {
			left_path = base_path + "measurements/Dev0_Image_w1920_h1200_fn" + to_string(index) + ".bmp";
			right_path = base_path + "measurements/Dev1_Image_w1920_h1200_fn" + to_string(index) + ".bmp";
		}
        else
        {
            left_path = base_path + "measurements/Dev1_Image_w1920_h1200_fn" + to_string(index) + ".bmp";
			right_path = base_path + "measurements/Dev0_Image_w1920_h1200_fn" + to_string(index) + ".bmp";
        }
        Mat left = imread(left_path, -1);
        Mat right = imread(right_path, -1);
        Size origSize = left.size();

        // read pointcloud 
        scan_path = base_path + "scans/test_fn" + to_string(index) + ".ply";
        new_scan_path = base_path + "scans/left_d" + to_string(new_index) + ".ply";
        // resave under different name
        //resavePLY(scan_path, new_scan_path);
        
        // remove distortion
        Mat combinedRemap1(Size(newSize.width*2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
        SS.getImage(0, SurroundSystem::RECTIFIED, left, right, combinedRemap1);

        imwrite(base_path +"left/left_d" + to_string(new_index) + ".png", combinedRemap1(Rect(0, 0, newSize.width, newSize.height)));
        imwrite(base_path +"right/right_d" + to_string(new_index) + ".png", combinedRemap1(Rect(newSize.width, 0, newSize.width, newSize.height)));

        cout << new_index <<  " Index done" << endl;
        new_index++;
    }
    cout << "Measurements done, prcoessing calibration images..." << endl;
    // loop over all calibration images
    for (int i = 1; i <= num_distances; i++)
    {
        string left_path, right_path, scan_path, new_scan_path;
        if (swap_lr)
        {
            left_path = base_path + "calibration/Dev0_Image_w1920_h1200_fn" + to_string(i) + ".bmp";
            right_path = base_path + "calibration/Dev1_Image_w1920_h1200_fn" + to_string(i) + ".bmp";
        }
        else
        {
            left_path = base_path + "calibration/Dev1_Image_w1920_h1200_fn" + to_string(i) + ".bmp";
            right_path = base_path + "calibration/Dev0_Image_w1920_h1200_fn" + to_string(i) + ".bmp";
        }
        Mat left = imread(left_path, -1);
        Mat right = imread(right_path, -1);
        Size origSize = left.size();

        // remove distortion
        Mat combinedRemap1(Size(newSize.width * 2, newSize.height), CV_8UC3, Scalar(0, 0, 0));
        SS.getImage(0, SurroundSystem::RECTIFIED, left, right, combinedRemap1);

        imwrite(base_path + "left/left_fn" + to_string(i) + ".png", combinedRemap1(Rect(0, 0, newSize.width, newSize.height)));
        imwrite(base_path + "right/right_fn" + to_string(i) + ".png", combinedRemap1(Rect(newSize.width, 0, newSize.width, newSize.height)));

        cout << i << " Index done" << endl;
	}
    cout << "All done" << endl;

	return 0;

}
