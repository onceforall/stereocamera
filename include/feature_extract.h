#pragma once
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <vector>

using namespace cv;
class SIFT_EXTRACT
{
public:
      SIFT_EXTRACT(){};
      virtual ~SIFT_EXTRACT(){};
      std::vector<KeyPoint> keypointsL,keypointsR;
      Mat descriptorsL,descriptorsR;
      std::vector<std::vector<DMatch>> match;
      std::vector<DMatch> goomatch;
      std::vector<Point2f> psL,psR;
      void getdsp(Mat imgL,Mat imgR);
      void goodmatcher(Mat imgL,Mat imgR);
};