#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <vector>
#include <opencv2/features2d.hpp>
using namespace cv;
using namespace std;

class SIFT_EXTRACT
{
public:
     
      SIFT_EXTRACT(){};
      virtual ~SIFT_EXTRACT(){};
      vector<KeyPoint> keypointsL,keypointsR;
      Mat descriptorsL,descriptorsR;
      vector<vector<DMatch>> match;
      vector<DMatch> goomatch;
      vector<Point2f> psL,psR;
      void getdsp(Mat imgL,Mat imgR);
      void goodmatcher(Mat imgL,Mat imgR);
};

 void SIFT_EXTRACT::getdsp(Mat imgL,Mat imgR)
 {
     Mat imgl,imgr;
     GaussianBlur(imgL,imgl,Size(3,3),0.5);
     GaussianBlur(imgR,imgr,Size(3,3),0.5);
     Ptr<Feature2D> sift=xfeatures2d::SURF::create();
     sift->detect(imgl,keypointsL);
     sift->detect(imgr,keypointsR);
     sift->compute(imgl,keypointsL,descriptorsL); 
     sift->compute(imgr,keypointsR,descriptorsR);
 }

void  SIFT_EXTRACT::goodmatcher(Mat imgL,Mat imgR)
{
    using namespace std;
    const int k=2;
    const float ratio=1.f/1.5f;
    BFMatcher matcher(NORM_L2,false);
    vector<DMatch> goodmatch;
    Mat matchimg;
    matcher.knnMatch(descriptorsL,descriptorsR,match,2);
    for(size_t i=0;i<match.size();i++)
    {
        const DMatch& bestmatch=match[i][0];
        const DMatch& bettermatch=match[i][1];
        float distanceRatio=bestmatch.distance/bettermatch.distance;
        if(distanceRatio<ratio)
            goodmatch.push_back(bestmatch);
    }
    std::sort(goodmatch.begin(), goodmatch.end());
    if(goodmatch.size()>4)
    {
        for(size_t i=0;i<goodmatch.size()*0.3;i++)
        {
            psL.push_back(keypointsL[goodmatch[i].queryIdx].pt);
            psR.push_back(keypointsR[goodmatch[i].trainIdx].pt);
        }
        goodmatch.clear();
        drawMatches(imgL,keypointsL,imgR,keypointsR,goodmatch,matchimg,(0,255,0),(255,0,0));
        imwrite("/home/yons/projects/stereocamera/res/match.png",matchimg);
        
        
    }
    
}

