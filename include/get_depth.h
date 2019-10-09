#pragma once
#include <iostream>
#include <string>
#include <map> 
using namespace std;
#include "opencv2/calib3d.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <thread>
#include <future>
#include <utility>
#include <chrono>

void get_depth(cv::VideoCapture cap,map<string,string> filename,cv::Mat* framel,cv::Mat*framer,int index,bool no_display,float* depth);