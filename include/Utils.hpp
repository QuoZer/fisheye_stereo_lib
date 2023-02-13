#pragma once

#include "opencv2/ccalib/omnidir.hpp"
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/highgui.hpp"
#include <vector>
#include <iostream>
#include <string>
#include <map>
#include <cstdarg>
#include "SurroundSystem.hpp"

void cropUglyScreenshots(const std::vector<std::string>& list);

// beta = brightness, alpha = contrast
void changeContrastAndBrightness(cv::Mat& image, double alpha, int beta);

// show a matrix of n images
void ShowManyImages(std::string title, int nArgs, ...);

// take two images, dewarp them with all the configured models and save the resultig images
void saveWithAllModels(std::string base_folder, SurroundSystem& SS, cv::Mat& left, cv::Mat& right, int index);

void savePano(std::string base_folder, SurroundSystem& SS, cv::Mat& combinedRemap, int type_i, int sp_index);

bool readStringList(const std::string& filename, std::vector<std::string>& l);