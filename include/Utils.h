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

void ShowManyImages(std::string title, int nArgs, ...);

void saveWithAllModels(SurroundSystem& SS, cv::Mat& left, cv::Mat& right, int index);

void savePano(SurroundSystem& SS, cv::Mat& combinedRemap, int type_i, int sp_index);

bool readStringList(const std::string& filename, std::vector<std::string>& l);