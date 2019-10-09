#pragma once
#include<opencv2/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/xfeatures2d.hpp>
#include<vector>
#include <opencv2/stitching.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <string>

void image_stitch(cv::Mat* frameadjust,cv::Mat* framestitch,cv::Mat* frameLr,cv::Mat* frameRr);
void static_stitching(std::string imgL,std::string imgR);