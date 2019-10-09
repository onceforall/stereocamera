#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
void adjustimg(cv::Mat* frameL,cv::Mat* frameR,cv::Mat* frameadjust,cv::Mat* framestitch,float* dpl,float* dpr);