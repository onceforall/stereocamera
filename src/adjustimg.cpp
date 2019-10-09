#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include "adjustimg.h"
#include <iostream>
#include <string>

extern bool ischanged;

void adjustimg(cv::Mat* frameL,cv::Mat* frameR,cv::Mat* frameadjust,cv::Mat* framestitch,float* dpl,float* dpr)
{
    while(true)
    {
        float ratio=1;
        cv::Mat res;
        cv::Mat resized;
        cv::Size size=frameL->size();
       
        if(*dpl!=0 && *dpr!=0)
        {
            //std::cout<<"DEPTH:"<<*dpl<<' '<<*dpr<<' '<<*dpl/(*dpr)<<std::endl;
            ratio=*dpl/(*dpr);
            //std::cout<<"ratio:"<<ratio<<std::endl;
            if(ratio>=0.8 && ratio<1 ) 
            {
                ischanged=true;
                static int count=0;
                cv::Size newsize(size.width*1/ratio,size.height*1/ratio);
                cv::resize(*frameR,resized,newsize,1,1,1);
                std::cout<<"size:"<<ratio<<' '<<resized.size()<<' '<<size<<std::endl;
                res=resized(cv::Rect((int)((frameR->size[1]-size.width))/4,(int)((frameR->size[0]-size.height))/4,size.width,size.height));
                res.copyTo(*frameadjust);
                frameL->copyTo(*framestitch);
                std::cout<<"adjudt.size:"<<frameadjust->size()<<std::endl;
                cv::imwrite("/home/yons/projects/stereocamera/res/resR.png",res);
                cv::imwrite("/home/yons/projects/stereocamera/res/resL.png",*frameL);
                cv::imwrite("/home/yons/projects/stereocamera/res/ratio1_frameR.png",*frameR);
                cv::imwrite("/home/yons/projects/stereocamera/res/ratio0_frameL.png",*frameL);
                //cv::imshow("RESR",res);
                //cv::imshow("RESL",*frameL);
                //cv::waitKey(30);

                //break;
            }
            else if(ratio>1 && ratio<=1.2)
            {
                ischanged=true;
                static int num=0;
                cv::Size newsize(size.width*ratio,size.height*ratio);
                cv::resize(*frameL,resized,newsize,1,1,1);
                std::cout<<"size:"<<ratio<<' '<<resized.size()<<' '<<size<<std::endl;
                res=resized(cv::Rect((int)((frameL->size[1]-size.width))/4,(int)((frameL->size[0]-size.height))/4,size.width,size.height));
                res.copyTo(*frameadjust);
                frameR->copyTo(*framestitch);
                std::cout<<"adjudt.size:"<<frameadjust->size()<<std::endl;
                //cv::imshow("RESL",res);
                //cv::imshow("RESR",*frameR);
                //cv::waitKey(30);
                cv::imwrite("/home/yons/projects/stereocamera/res/resL.png",res);
                cv::imwrite("/home/yons/projects/stereocamera/res/resR.png",*frameR);
                cv::imwrite("/home/yons/projects/stereocamera/res/ratio0_frameL.png",*frameL);
                cv::imwrite("/home/yons/projects/stereocamera/res/ratio1_frameR.png",*frameR);
                //break;
            }
            else
            {
                //cv::imwrite("/home/yons/projects/stereocamera/res/resL.png",*frameL);
                //cv::imwrite("/home/yons/projects/stereocamera/res/resR.png",*frameR);  
                cv::imwrite("/home/yons/projects/stereocamera/res/ratio0_frameL.png",*frameL);
                cv::imwrite("/home/yons/projects/stereocamera/res/ratio1_frameR.png",*frameR);    
                //cv::waitKey(30);   
                continue;     
            }
        }
    }
}