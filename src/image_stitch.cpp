#include <iostream>
#include <fstream>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/stitching.hpp>
#include <string>
#include <opencv2/core/core.hpp>


extern bool ischanged;
int i=0,j=0;
void image_stitch(cv::Mat* frameadjust,cv::Mat* framestitch,cv::Mat* frameLr,cv::Mat* frameRr)
{
    using namespace std;
    using namespace cv;
    bool try_use_gpu = true;
    vector<Mat> imgs;
    double minv=-1.0,maxv=-1.0;
    double *minp=&minv;
    double *maxp=&maxv;
    string result_depth = "/home/yons/projects/stereocamera/res/result_dep_";
    string result_regular="/home/yons/projects/stereocamera/res/result_";
    Mat pano;
    Stitcher stitcher = Stitcher::createDefault(try_use_gpu);
    Size framesize(1280,720);
    while(true)
    {
        //cv::Mat origL      =cv::imread("/home/yons/projects/stereocamera/res/ratio0_frameL.png");
        //cv::Mat frameL     =cv::imread("/home/yons/projects/stereocamera/res/resL.png");
        //cv::Mat frameadjust=cv::imread("/home/yons/projects/stereocamera/res/resR.png");
        //cv::Mat origR      =cv::imread("/home/yons/projects/stereocamera/res/ratio1_frameR.png");
        imgs.clear();
       
        //if(!frameLr->empty())     imgs.push_back(*frameLr);
        if(ischanged)
        {
            ischanged=false;
            minMaxIdx(*frameadjust,minp,maxp);
            if(maxv==0) continue;
            minMaxIdx(*framestitch,minp,maxp);
            if(maxv==0) continue;
            if(!frameadjust->empty()) imgs.push_back(*frameadjust);
            if(!framestitch->empty()) imgs.push_back(*framestitch);
            

            //if(!frameRr->empty())     imgs.push_back(*frameRr);
            
            Stitcher::Status status = stitcher.stitch(imgs, pano);
            if (status != Stitcher::OK)
            {
                cout << "Can't stitch images, error code = " << int(status) << endl;
                
            }
            
            else
            {
                cout<<"stitching depth"<<endl;
                cv::resize(pano,pano,framesize,0,0,1);
                imwrite(result_depth+to_string(j++)+".png", pano);
                cv::imshow("result",pano);
                waitKey(30);
            }
            
        }
        else
        {
            
            imgs.clear();
            minMaxIdx(*frameLr,minp,maxp);
            if(maxv==0) continue;
            minMaxIdx(*frameRr,minp,maxp);
            if(maxv==0) continue;
            if(!frameLr->empty())     imgs.push_back(*frameLr);
            if(!frameRr->empty())     imgs.push_back(*frameRr);
            Stitcher::Status status = stitcher.stitch(imgs, pano);
            if(status!=Stitcher::OK)
                continue;
            else
            { 
                cout<<"stitching original"<<endl;
                cv::resize(pano,pano,framesize,0,0,1);
                imwrite(result_regular+to_string(i++)+".png", pano);
                cv::imshow("result",pano);
                waitKey(30);
            }
        }
        
	         
    }
}

using namespace std;
using namespace cv;
void static_stitching(string imgL,string imgR)
{
  
    bool try_use_gpu = true;
    vector<Mat> imgs;
    string result_name = imgL.substr(0,imgL.find_last_not_of('.'))+"_result.png";
    Mat frameL=cv::imread(imgL);
    Mat frameR=cv::imread(imgR);
    Mat pano;
    Stitcher stitcher = Stitcher::createDefault(try_use_gpu);
    Size framesize(1280,640);
  
    imgs.clear();
       
        
    if(!frameL.empty()) imgs.push_back(frameL);
    if(!frameR.empty()) imgs.push_back(frameR);
   
        
       
    Stitcher::Status status = stitcher.stitch(imgs, pano);
    if (status != Stitcher::OK)
    {
        cout << "Can't stitch images, error code = " << int(status) << endl;
    }
    else
    {
        cout<<"stitching"<<endl;
        cv::resize(pano,pano,framesize,0,0,1);
        cv::imshow("result",pano);
        imwrite(result_name, pano);
    }
}

