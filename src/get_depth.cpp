
#include <iostream>
#include "opencv2/calib3d.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>
#include <stdio.h>
#include <string>
#include <fstream>
#include <map>
#include <vector>
#include <boost/typeof/typeof.hpp>
#include "feature_extract.h"
#include "get_depth.h"

using namespace std;
using namespace cv;


const int frameWidth=2560;
const int frameHeight=720;

Size frameSize=Size(1280,360);
Size imageSize(640,360);
bool selectObject=false;                       /*choose or not*/
int windowsize=3;
int min_disp=2;
int nums_disp=130-min_disp;

int minDisparity=0,numDisparities=16,blockSize=windowsize,P1=8*3*pow(windowsize,2),P2=32*3*pow(windowsize,2),disp12MaxDiff=5,preFilterCap=0,uniquenessRatio=10,\
		 speckleWindowSize=100,speckleRange=32;
float scale=1.0f;
int lambda=80000;
float sigma=1.8;
#if 0
static void saveXYZ(const char* filename,const Mat& mat)
{
	const double max_z=1.0e4;
	FILE* fp=fopen(filename,"wt");
	for(int y=0;y<mat.rows;y++)
	{
            for(int x=0;x<mat.cols;x++)
	    {
	        Vec3f point=mat.at<Vec3f>(y,x);
		if(fabs(point[2]-max_z)<FLT.EPSILON || fabs(point[2]>max_z)) continue;
		fprintf(fp,"%f %f %f\n",point[0],point[1],point[2]);
	    }
	}
	fclose(fp);
}
#endif
void readMatrixfromTXT(string fileName, const int numRow,const int numColumn,  Mat* matrix)
{
	// std::ifstream fin(fileName,std::ifstream::in);
	ifstream fin(fileName, ios::in);
	// ifstream fin(fileName.c_str(),ios::in);
	if (!fin)
	{
		std::cout<<fileName<<std::endl;
		cerr << "不能打开文件" << endl;
		exit(1);
	}
	cout<<fileName<<endl;
	string line;
	double tmp;
	//int j = 0;
	for (int i = 0; i<numRow; i++)
	{
		getline(fin, line);
		//j = 0;
		//for(int j=0;j<numColumn;j++){
		istringstream istr(line);
		// float* pdata=(float *)(matrix->data+i*matrix->step);
		// while (istr >> tmp)
		// {
		// 	*pdata= tmp;
		// 	pdata++;
		// 	//++j;
		// 	cout<<tmp<<' ';
		// }
		for(int j=0;j<numColumn;j++)
		{
			istr>>matrix->at<double>(i,j);
			cout<<matrix->at<double>(i,j)<<' ';
		}
		cout<<endl;
		istr.clear();
		line.clear();
	}
	
	fin.close();
}


void get_depth(VideoCapture cap,map<string,string> filename,cv::Mat* framel,cv::Mat*framer,int index,bool display,float* depth)
{
	
	Mat frame;
	Mat rgbframeL,grayframeL;                      /*original frameL,frameR*/
	Mat rgbframeR,grayframeR;
	Mat rectifiedframeL,rectifiedframeR;           /*rectified frameL,frameR*/
	Mat rgbrectifiedframeL,rgbrectifiedframeR;
	Rect ValidROIL;                                /*ROI region*/
	Rect ValidROIR;

	Mat* Ml=new Mat(3,3, CV_64F);
	Mat* Dl=new Mat(1,5, CV_64F);
	Mat* Mr=new Mat(3,3, CV_64F);
	Mat* Dr=new Mat(1,5, CV_64F);                               /*Ml,Mr: camera instrinsic matrix, Dl,Dr:Camera distortion coefficients*/
	Mat mapLx;
	Mat mapLy;
	Mat mapRx;
	Mat mapRy;
	Mat Rm = cv::Mat::eye(3, 3, CV_32F);                /*map matrix*/
	Mat Rl,Rr,Pl,Pr,Q;    
	Mat* R=new Mat(3,3,CV_64F,Scalar::all(1));
	Mat* T=new Mat(3,1,CV_64F,Scalar::all(1));                         /*rotation matrix,translation matrix,map matrix and remapping matrix*/
	Mat xyz;                                       /*3d world coords*/
	Mat dispL,dispR;                                      /*disparity map*/
	Point origin;                                  /*mouse click point*/
	Rect selected;
	Size rec_size(640,360);
	//cv::Ptr<cv::StereoSGBM> sgbm=cv::StereoSGBM::create(0,16,3);
	
	readMatrixfromTXT(filename["Ml"],3,3,Ml);
	readMatrixfromTXT(filename["Dl"],1,5,Dl);
	readMatrixfromTXT(filename["Mr"],3,3,Mr);
	readMatrixfromTXT(filename["Dr"],1,5,Dr);
	
	readMatrixfromTXT(filename["R"],3,3,R);
	readMatrixfromTXT(filename["T"],3,1,T);

	Ptr<StereoSGBM> left_matcher  = StereoSGBM::create(minDisparity,
		    numDisparities,
		    blockSize,
		    P1,P2,
		    disp12MaxDiff,
		    preFilterCap,
		    uniquenessRatio,
		    speckleWindowSize,
		    speckleRange);
    
	Ptr<StereoSGBM> right_matcher  = StereoSGBM::create(minDisparity,
		    numDisparities,
		    blockSize,
		    P1,P2,
		    disp12MaxDiff,
		    preFilterCap,
		    uniquenessRatio,
		    speckleWindowSize,
		    speckleRange);

	SIFT_EXTRACT sift;	
	Ptr<ximgproc::DisparityWLSFilter> wls_filter;
   
    if(!cap.isOpened())
	{
		 return ;
	}
	
	
	
	
    while(true)
    {
		Mat frame_O;
    	cap.read(frame_O);
		resize(frame_O,frame,frameSize,0.5,0.5,1);                                         //resize frame by 0.5
		rgbframeL=frame(Rect(0,0,frame.size[1]/2,frame.size[0]));
		rgbframeR=frame(Rect(frame.size[1]/2,0,frame.size[1]/2,frame.size[0]));
		if(rgbframeL.empty() || rgbframeR.empty())
		 	return ;
		// if(Ml->size[0] && Ml->size[1])
		// {
		// 	for(int row=0;row< Ml->size[0];row++)
		// 	{
		// 		float* pdata=(float*)( Ml->data+row* Ml->step);
		// 		for(int col=0;col< Ml->size[1];col++)
		// 		{
		// 			cout<<*pdata<<endl;
		// 			pdata++;
		// 		}
				
		// 	}
		// 	exit(-1);
		// }
	
		#if 0
        if(scale!=1.f)
	    {
            Mat temp1,temp2;
	        int method=scale<1?INTER_AREA:INTER_CUBIC;
	        resize(rgbframeL,temp1,Size(),scale,scale,method);
	        resize(rgbframeR,temp2,Size(),scale,scale,method);
            rgbframeL=temp1;
	        rgbframeR=temp2;
	    }
	    Size frame_size=rgbframeL.size();
         /* get the instrinsic matrix and extrinsic matrix from file/csv file */
#if 0
	 FileStorage fs(instrinsic_filename,FileStorage::READ);
	 if(!fs.isOpened())
	 {
	     printf("Failed to open  file%s\n",instrinsic_filename);
	 }
#endif
        cd
		
		//waitKey();
		#endif
		//rectifiedframeL=rgbframeL.clone();
		//rectifiedframeR=rgbframeR.clone();
		

		stereoRectify(*Ml,*Dl,*Mr,*Dr,imageSize,*R,*T,Rl,Rr,Pl,Pr,Q,CALIB_ZERO_DISPARITY,-1,rec_size,&ValidROIL,&ValidROIR);     //input:Ml,Dl,Mr,Dr,R,T   OUTPUT:Rl,Rr,Pl,Pr,Q
		initUndistortRectifyMap(*Ml,*Dl,Rl,Mat(),rec_size,CV_32FC1,mapLx,mapLy);
		initUndistortRectifyMap(*Mr,*Dr,Rr,Mat(),rec_size,CV_32FC1,mapRx,mapRy);
	
		remap(rgbframeL,rectifiedframeL,mapLx,mapLy,INTER_LINEAR);
		remap(rgbframeR,rectifiedframeR,mapRx,mapRy,INTER_LINEAR);

		//undistort(rgbframeL,rectifiedframeL,*Ml,*Dl);
		//undistort(rgbframeR,rectifiedframeR,*Mr,*Dr);
		//imshow("原始图像", rgbframeL);
        //imshow("矫正后图像", rectifiedframeL);
		//cv::waitKey(30);
		
		rectifiedframeL.copyTo(*framel);
		rectifiedframeR.copyTo(*framer);
		
								   
		cvtColor(rectifiedframeL,grayframeL,CV_BGR2GRAY);                                                                          //BGR2GRAY
		cvtColor(rectifiedframeR,grayframeR,CV_BGR2GRAY);

		sift.getdsp(grayframeL,grayframeR);		                        										       //get the descriptors
		sift.goodmatcher(grayframeL,grayframeR);	                                                                   // get the good matchers
	
		int64 t=getTickCount();
		double matching_time = (double)getTickCount();
    	left_matcher-> compute(grayframeL, grayframeR, dispL);
    	right_matcher->compute(grayframeR, grayframeL, dispR);
    	matching_time = ((double)getTickCount() - matching_time)/getTickFrequency();

		wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
		wls_filter->setLambda(lambda);
    	wls_filter->setSigmaColor(sigma);
		Mat filtered_disp;
		double filtering_time = (double)getTickCount();
        wls_filter->filter(dispL,rectifiedframeL,filtered_disp,dispR);
    	double filtered_time = ((double)getTickCount() - filtering_time)/getTickFrequency();

    	Mat conf_map = wls_filter->getConfidenceMap();

    	//Get the ROI that was used in the last filter call:
    	Rect ROI = wls_filter->getROI();
    	if(scale!=1.0f)
    	{
        	//upscale raw disparity and ROI back for a proper comparison:
        	resize(dispL,dispL,Size(),2.0,2.0,1);
        	dispL = dispL*2.0;
        	ROI = Rect(ROI.x*2,ROI.y*2,ROI.width*2,ROI.height*2);
    	}
		
		Mat filtered_color;
		Mat disp8;                                              /*16 bit -->8bit*/
		//normalize(filtered_disp,disp8,0,255,CV_MINMAX);
    	filtered_disp.convertTo(disp8, CV_8U, 255/(numDisparities*16.));
		
		//applyColorMap(disp8,filtered_color,COLORMAP_RAINBOW);
        putText(disp8,to_string((int)(1/(matching_time+filtered_time))),Point2d(50,50),FONT_HERSHEY_TRIPLEX,2,(255,255,255),1,false);
		
		//typedef BOOST_TYPEOF(*disp8.data) ElementType;
		//int channels=filtered_color.channels();
		
		//CV_Assert(channels==3);
		
		float avedepth=0;
		float baseline=60;
		uchar* dispData = (uchar*)disp8.data;
		if(sift.psL.size() && sift.psR.size())
		{
			int row=-1,col=-1;
			for(size_t i=0;i<sift.psL.size();i++)
			{
				for(int r=-1;r<=1;r++)
				{

					for(int c=-1;c<=1;c++)
					{
						row=sift.psL[i].x+r;
						col=sift.psL[i].y+c;
						if(row>=rec_size.height) row=rec_size.height-1;
						if(row<0) row=0;
						if(col>=rec_size.width) col=rec_size.width-1;
						if(col<0) col=0;
						int id = r*disp8.size[0] + c;
						if(!dispData[id]) continue;
						float used_depth=Ml->at<double>(0,0)*baseline/(float)dispData[id];
						
						//cout<<"depth "<<used_depth<<endl;
						avedepth+=used_depth/9;
					}
				}
			}
			*depth=avedepth;
			//std::cout<<"sift feature size: "<<sift.psL.size()<<" "<<sift.psR.size()<<std::endl;
		}
		//promiseobj.set_value(depth);
		if(display)
		{
			namedWindow("origin"+to_string(index),1);
			imshow("origin"+to_string(index),frame);
        	// namedWindow("left"+to_string(index),1);
	    	// imshow("left"+to_string(index),rectifiedframeL);
	    	// namedWindow("right"+to_string(index),1);
        	// imshow("right"+to_string(index),rectifiedframeR);
	    	// namedWindow("disparity"+to_string(index),WINDOW_AUTOSIZE);
			//imshow("disparity"+to_string(index),disp8);
			waitKey(30);
        	//printf("press any key to continue...");
	    	//fflush(stdout);
	    	//printf("\n");
		}

		if(!filename["Disparity"].empty())
		{
			imwrite(filename["Disparity"]+to_string(index)+".png",disp8);
		}
	

	
#if 0
	Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
	sgbm->setPreFilterCap(63);
    int sgbmWinSize = blockSize > 0 ? blockSize : 3;
    sgbm->setBlockSize(sgbmWinSize);

    int cn = rgbframeL.channels();

    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    
    sgbm->setMode(StereoSGBM::MODE_SGBM);
   

    Mat disp, disp8;
    //Mat img1p, img2p, dispp;
    //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

    int64 t = getTickCount();
    
    sgbm->compute(rectifiedframeL, rectifiedframeR, disp);
    t = getTickCount() - t;
    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

    //disp = dispp.colRange(numberOfDisparities, img1p.cols);
   
    disp.convertTo(disp8, CV_8U, 255/(numDisparities*16.));
    no_display=false;
    if( !no_display )
    {
	
        namedWindow("left", 1);
        imshow("left", rectifiedframeL);
        namedWindow("right", 1);
        imshow("right", rectifiedframeR);
        namedWindow("disparity", 0);
        imshow("disparity", disp8);
		cout<<disp8.at<float>(180,180)<<endl;
        printf("press any key to continue...");
        fflush(stdout);
        waitKey(30);
        printf("\n");
    }
#endif

#if 0
	 if(!point_cloud_filename.c_str().empty())
	 {

             printf("storing the point cloud...");
	     fflush(stdout);
	     reprojectImageTo3D(disp,xyz,Q,true);
	     saveXYZ(point_cloud_filename,xyz);
	     printf("\n");
	 }
#endif
	//return 0;
    }
}



int get_dep(string img1_filename,string img2_filename,string intrinsic_filename,string extrinsic_filename )
{
    
    enum { STEREO_BM=0, STEREO_SGBM=1, STEREO_HH=2, STEREO_VAR=3, STEREO_3WAY=4 };
    int alg = STEREO_SGBM;
    int SADWindowSize, numberOfDisparities;
    bool no_display;
    float scale;

    Ptr<StereoBM> bm = StereoBM::create(16,9);
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
   
   

    int color_mode = alg == STEREO_BM ? 0 : -1;
    Mat img1 = imread(img1_filename, color_mode);
    Mat img2 = imread(img2_filename, color_mode);

    if (img1.empty())
    {
        printf("Command-line parameter error: could not load the first input image file\n");
        return -1;
    }
    if (img2.empty())
    {
        printf("Command-line parameter error: could not load the second input image file\n");
        return -1;
    }

    if (scale != 1.f)
    {
        Mat temp1, temp2;
        int method = scale < 1 ? INTER_AREA : INTER_CUBIC;
        resize(img1, temp1, Size(), scale, scale, method);
        img1 = temp1;
        resize(img2, temp2, Size(), scale, scale, method);
        img2 = temp2;
    }

    Size img_size = img1.size();

    Rect roi1, roi2;
    Mat Q;

    if( !intrinsic_filename.empty() )
    {
        // reading intrinsic parameters
        FileStorage fs(intrinsic_filename, FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", intrinsic_filename.c_str());
            return -1;
        }

        Mat M1, D1, M2, D2;
        fs["M1"] >> M1;
        fs["D1"] >> D1;
        fs["M2"] >> M2;
        fs["D2"] >> D2;

        M1 *= scale;
        M2 *= scale;

        fs.open(extrinsic_filename, FileStorage::READ);
        if(!fs.isOpened())
        {
            printf("Failed to open file %s\n", extrinsic_filename.c_str());
            return -1;
        }

        Mat R, T, R1, P1, R2, P2;
        fs["R"] >> R;
        fs["T"] >> T;

        stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

        Mat map11, map12, map21, map22;
        initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
        initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

        Mat img1r, img2r;
        remap(img1, img1r, map11, map12, INTER_LINEAR);
        remap(img2, img2r, map21, map22, INTER_LINEAR);

        img1 = img1r;
        img2 = img2r;
    }

    numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;

    bm->setROI1(roi1);
    bm->setROI2(roi2);
    bm->setPreFilterCap(31);
    bm->setBlockSize(SADWindowSize > 0 ? SADWindowSize : 9);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(1);

    sgbm->setPreFilterCap(63);
    int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;
    sgbm->setBlockSize(sgbmWinSize);

    int cn = img1.channels();

    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    if(alg==STEREO_HH)
        sgbm->setMode(StereoSGBM::MODE_HH);
    else if(alg==STEREO_SGBM)
        sgbm->setMode(StereoSGBM::MODE_SGBM);
    else if(alg==STEREO_3WAY)
        sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);

    Mat disp, disp8;
    //Mat img1p, img2p, dispp;
    //copyMakeBorder(img1, img1p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);
    //copyMakeBorder(img2, img2p, 0, 0, numberOfDisparities, 0, IPL_BORDER_REPLICATE);

    int64 t = getTickCount();
    if( alg == STEREO_BM )
        bm->compute(img1, img2, disp);
    else if( alg == STEREO_SGBM || alg == STEREO_HH || alg == STEREO_3WAY )
        sgbm->compute(img1, img2, disp);
    t = getTickCount() - t;
    printf("Time elapsed: %fms\n", t*1000/getTickFrequency());

    //disp = dispp.colRange(numberOfDisparities, img1p.cols);
    if( alg != STEREO_VAR )
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
    else
        disp.convertTo(disp8, CV_8U);
    if( !no_display )
    {
        namedWindow("left", 1);
        imshow("left", img1);
        namedWindow("right", 1);
        imshow("right", img2);
        namedWindow("disparity", 0);
        imshow("disparity", disp8);
        printf("press any key to continue...");
        fflush(stdout);
        waitKey();
        printf("\n");
    }
#if 0
    //if(!disparity_filename.empty())
        imwrite(disparity_filename, disp8);

    if(!point_cloud_filename.empty())
    {
        printf("storing the point cloud...");
        fflush(stdout);
        Mat xyz;
        reprojectImageTo3D(disp, xyz, Q, true);
        saveXYZ(point_cloud_filename.c_str(), xyz);
        printf("\n");
    }
#endif
    return 0;
}