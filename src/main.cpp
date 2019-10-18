#include"get_depth.h"
#include<vector>
#include<string>
#include<map>
#include<thread>
#include<opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include<iostream>
#include "adjustimg.h"
#include "image_stitch.h"
#include "calibration.h"
#include "wrapimg.hpp"

using namespace std;
using namespace cv;


vector<int> cam_index;
vector<int> countCameras()
{
    VideoCapture temp_camera;
    int maxTested = 10;
    for (int i = 0; i < maxTested; i++)
    {
        VideoCapture temp_camera(i);
        bool res = (!temp_camera.isOpened());
        temp_camera.release();
        if (res)
        {
            cam_index.push_back(i);
        }
    }
   return cam_index;
}

bool ischanged=false;
int main()
{
    
    const int height=720;
    const int width=2560;
    float depthL=1.0,depthR=1.0;
    float* dpl=&depthL;
    float* dpr=&depthR;
    cv::Mat frameLl(height/2,width/4,CV_8UC3);
    cv::Mat frameRl(height/2,width/4,CV_8UC3);
    cv::Mat frameLr(height/2,width/4,CV_8UC3);
    cv::Mat frameRr(height/2,width/4,CV_8UC3);
    cv::Mat frameadjust(height/2,width/4,CV_8UC3);
    cv::Mat framestitch(height/2,width/4,CV_8UC3);
    map<string,string> filenames;
    filenames["Ml"]="/home/yons/projects/stereocamera/640×360/c_m_l1.txt";
    filenames["Dl"]="/home/yons/projects/stereocamera/640×360/c_d_l1.txt";
    filenames["Mr"]="/home/yons/projects/stereocamera/640×360/c_m_r1.txt";
    filenames["Dr"]="/home/yons/projects/stereocamera/640×360/c_d_r1.txt";
    filenames["R"]="/home/yons/projects/stereocamera/640×360/R1.txt";
    filenames["T"]="/home/yons/projects/stereocamera/640×360/T1.txt";
    filenames["Disparity"]="/home/yons/projects/stereocamera/res/depth";

    map<string,string> filenames2;
    filenames2["Ml"]="/home/yons/projects/stereocamera/640×360/c_m_l2.txt";
    filenames2["Dl"]="/home/yons/projects/stereocamera/640×360/c_d_l2.txt";
    filenames2["Mr"]="/home/yons/projects/stereocamera/640×360/c_m_r2.txt";
    filenames2["Dr"]="/home/yons/projects/stereocamera/640×360/c_d_r2.txt";
    filenames2["R"]="/home/yons/projects/stereocamera/640×360/R2.txt";
    filenames2["T"]="/home/yons/projects/stereocamera/640×360/T2.txt";
    filenames2["Disparity"]="/home/yons/projects/stereocamera/res/depth";
    thread threads[4];
    //countCameras();
    //assert(cam_index.size()==2);

    
    cv::VideoCapture cap(0);
    //cap.set(CV_CAP_PROP_FOURCC,CV_FOURCC('M','J','P','G'));
    cap.set(CV_CAP_PROP_FRAME_HEIGHT,height);
    cap.set(CV_CAP_PROP_FRAME_WIDTH,width);
    cout<<cap.get(3)<<' '<<cap.get(4)<<endl;
    cap.set(CV_CAP_PROP_FPS,1);
    //cap.set(CV_CAP_PROP_FORMAT,CV_YUV2BGR_YUNV);
    double rate = cap.get(CV_CAP_PROP_FPS);
    cout << "FPS:" << rate << endl;
    
    cv::VideoCapture cap1(4);
    //cap1.set(CV_CAP_PROP_FOURCC,CV_FOURCC('M','J','P','G'));
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT,height);   
    cap1.set(CV_CAP_PROP_FRAME_WIDTH,width);
    cap1.set(CV_CAP_PROP_FPS,1);
    //cap1.set(CV_CAP_PROP_FORMAT,CV_YUV2BGR_YUNV);
    rate = cap1.get(CV_CAP_PROP_FPS);
    cout<<cap1.get(3)<<' '<<cap1.get(4)<<endl;
    cout << "FPS:" << rate << endl;
    string filename="/home/yons/projects/stereocamera/res/result_4.png";
    
    threads[0]=thread(get_depth,cap,filenames,&frameLl,&frameLr,0,false,dpl);
    threads[1]=thread(get_depth,cap1,filenames2,&frameRl,&frameRr,4,false,dpr);
    threads[2]=thread(adjustimg,&frameLl,&frameRl,&frameadjust,&framestitch,dpl,dpr);
    threads[3]=thread(image_stitch,&frameadjust,&framestitch,&frameLr,&frameRr);
    //threads[0]=thread(rectangle_via_wraping,filename);
    for(int i=0;i<4;i++) 
    { 
        cout<<"starting....."<<endl;
        threads[i].join();
    }
    //static_stitching("/home/yons/projects/stereocamera/res/resL.png","/home/yons/projects/stereocamera/res/resR.png");
    
    //static_stitching("/home/yons/projects/stereocamera/res/ratio0_frameL.png","/home/yons/projects/stereocamera/res/ratio1_frameR.png");
    
    //stereo_calibrate();
    return 0;
}