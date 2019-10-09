/*************************************************************************************
*
*   Description：相机标定，张氏标定法  单目标定
*   Author     ：JNU
*   Data       ：2018.7.22
*
************************************************************************************/
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
using namespace cv;
using namespace std;

std::vector<std::string>  filenames;



void calibrate(string filename,string outfilename,cv::Mat &cameraMatrix,cv::Mat &distCoeffs,std::vector<std::vector<Point2f>> &imagePointsSeq,std::vector<std::vector<cv::Point3f>> &objectPoints,int cnt)
{
    //保存文件名称
    
    bool calibrates=true;
    //需要更改的参数
    //左相机标定，指定左相机图片路径，以及标定结果保存文件
    //string infilename = "/home/yons/projects/stereocamera/sample/right.txt";        //如果是右相机把left改为right
    //string outfilename = "/home/yons/projects/stereocamera/sample/right1/caliberation_result.txt";

    //标定所用图片文件的路径,每一行保存一个标定图片的路径  ifstream 是从硬盘读到内存
    //ifstream fin(infilename);
    //保存标定的结果  ofstream 是从内存写到硬盘
    ofstream fout(outfilename);

    /*
    1.读取毎一幅图像，从中提取出角点，然后对角点进行亚像素精确化、获取每个角点在像素坐标系中的坐标
    像素坐标系的原点位于图像的左上角
    */
    std::cout << "开始提取角点......" << std::endl;;
    //图像数量
     int imageCount = 0;
    //图像尺寸
    cv::Size imageSize;
    //标定板上每行每列的角点数
    cv::Size boardSize = cv::Size(9, 6);
    //缓存每幅图像上检测到的角点
    std::vector<Point2f>  imagePointsBuf;
    //保存检测到的所有角点
    Size newsize(1280,720);
    
    if (calibrates)
    {
        int c=0;
        //读取完毕？
        while (c<cnt)
        {
            //一次读取一行
            //fin.getline(filename, sizeof(filename) / sizeof(char));
            //保存文件名
            //cout<<filename<<endl;
            string each_filename=filename+to_string(c++)+".jpg";
            cout<<each_filename<<endl;
            filenames.push_back(each_filename);
            //读取图片
            Mat imageInput = cv::imread(each_filename);
            cv::resize(imageInput,imageInput,newsize,2,2,1);
            cv::imshow("ori",imageInput);
            //读入第一张图片时获取图宽高信息
            if (imageCount == 0)
            {
                imageSize.width = imageInput.cols;
                imageSize.height = imageInput.rows;
                std::cout << "imageSize.width = " << imageSize.width << std::endl;
                std::cout << "imageSize.height = " << imageSize.height << std::endl;
            }

            std::cout << "imageCount = " << imageCount << std::endl;
            imageCount++;

            //提取每一张图片的角点
            if (cv::findChessboardCorners(imageInput, boardSize, imagePointsBuf) == 0)
            {
                //找不到角点
                std::cout << "Can not find chessboard corners!" << std::endl;
                exit(1);
            }
            else
            {
                Mat viewGray;
                //转换为灰度图片
                cv::cvtColor(imageInput, viewGray, cv::COLOR_BGR2GRAY);
                //亚像素精确化   对粗提取的角点进行精确化
                cv::find4QuadCornerSubpix(viewGray, imagePointsBuf, cv::Size(5, 5));
                //保存亚像素点
                imagePointsSeq.push_back(imagePointsBuf);
                //在图像上显示角点位置
                cv::drawChessboardCorners(viewGray, boardSize, imagePointsBuf, true);
                //显示图片
                //cv::imshow("Camera Calibration", viewGray);
                cv::imwrite("test.jpg", viewGray);
                //等待0.5s
                //waitKey(500);
            }
        }        
        
        //计算每张图片上的角点数 54
        int cornerNum = boardSize.width * boardSize.height;

        //角点总数
        int total = imagePointsSeq.size()*cornerNum;
        std::cout << "total = " << total << std::endl;

        for (int i = 0; i < total; i++)
        {
            int num = i / cornerNum;
            int p = i%cornerNum;
            //cornerNum是每幅图片的角点个数，此判断语句是为了输出，便于调试
            if (p == 0)
            {                                        
                std::cout << "\n第 " << num+1 << "张图片的数据 -->: " << std::endl;
            }
            //输出所有的角点
            std::cout<<p+1<<":("<< imagePointsSeq[num][p].x;
            std::cout << imagePointsSeq[num][p].y<<")\t";
            if ((p+1) % 3 == 0)
            {
                std::cout << std::endl;
            }
        }

        std::cout << "角点提取完成!" << std::endl;

        /*
        2.摄像机标定 世界坐标系原点位于标定板左上角(第一个方格的左上角)
        */
        std::cout << "开始标定" << std::endl;
        //棋盘三维信息，设置棋盘在世界坐标系的坐标
        //实际测量得到标定板上每个棋盘格的大小
        cv::Size squareSize = cv::Size(26, 26);
        //毎幅图片角点数量
        std::vector<int> pointCounts;
        
       
       
        //每幅图片的旋转向量
        std::vector<cv::Mat> tvecsMat;
        //每幅图片的平移向量
        std::vector<cv::Mat> rvecsMat;

        //初始化标定板上角点的三维坐标
        int i, j, t;
        for (t = 0; t < imageCount; t++)
        {
            std::vector<cv::Point3f> tempPointSet;
            //行数
            for (i = 0; i < boardSize.height; i++)
            {
                //列数
                for (j = 0; j < boardSize.width; j++)
                {
                    cv::Point3f realPoint;
                    //假设标定板放在世界坐标系中z=0的平面上。
                    realPoint.x = i*squareSize.width;
                    realPoint.y = j*squareSize.height;
                    realPoint.z = 0;
                    tempPointSet.push_back(realPoint);
                }
            }
            objectPoints.push_back(tempPointSet);
        }

        //初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板
        for (i = 0; i < imageCount; i++)
        {
            pointCounts.push_back(boardSize.width*boardSize.height);
        }
        //开始标定
        cv::calibrateCamera(objectPoints, imagePointsSeq, imageSize, cameraMatrix, distCoeffs, rvecsMat, tvecsMat);
        std::cout << "标定完成" << std::endl;
        //对标定结果进行评价
        std::cout << "开始评价标定结果......" << std::endl;
        //所有图像的平均误差的总和
        double totalErr = 0.0;
        //每幅图像的平均误差
        double err = 0.0;
        //保存重新计算得到的投影点
        std::vector<cv::Point2f> imagePoints2;
        std::cout << "每幅图像的标定误差:" << std::endl;
        fout << "每幅图像的标定误差:" << std::endl;
        for (i = 0; i < imageCount; i++)
        {
            std::vector<cv::Point3f> tempPointSet = objectPoints[i];
            //通过得到的摄像机内外参数，对空间的三维点进行重新投影计算，得到新的投影点imagePoints2（在像素坐标系下的点坐标）
            cv::projectPoints(tempPointSet, rvecsMat[i], tvecsMat[i], cameraMatrix, distCoeffs, imagePoints2);
            //计算新的投影点和旧的投影点之间的误差
            std::vector<cv::Point2f> tempImagePoint = imagePointsSeq[i];
            cv::Mat tempImagePointMat = cv::Mat(1, tempImagePoint.size(), CV_32FC2);
            cv::Mat imagePoints2Mat = cv::Mat(1, imagePoints2.size(), CV_32FC2);
            for (int j = 0; j < tempImagePoint.size(); j++)
            {
                imagePoints2Mat.at<cv::Vec2f>(0, j) = cv::Vec2f(imagePoints2[j].x, imagePoints2[j].y);
                tempImagePointMat.at<cv::Vec2f>(0, j) = cv::Vec2f(tempImagePoint[j].x, tempImagePoint[j].y);
            }
            //Calculates an absolute difference norm or a relative difference norm.
            err = cv::norm(imagePoints2Mat, tempImagePointMat, NORM_L2);
            totalErr += err /= pointCounts[i];
            std::cout << "  第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;
            fout<<  "第" << i + 1 << "幅图像的平均误差：" << err << "像素" << endl;

        }
        //每张图像的平均总误差
        std::cout << "  总体平均误差:" << totalErr / imageCount << "像素" << std::endl;
        fout << "总体平均误差:" << totalErr / imageCount << "像素" << std::endl;
        std::cout << "评价完成!" << std::endl;
        //保存标定结果
        std::cout << "开始保存标定结果....." << std::endl;
        //保存每张图像的旋转矩阵
        cv::Mat rotationMatrix = cv::Mat(3, 3, CV_32FC1, Scalar::all(0));
        fout << "相机内参数矩阵:" << std::endl;
        fout << cameraMatrix << std::endl << std::endl;
        fout << "畸变系数:" << std::endl;
        fout << distCoeffs << std::endl << std::endl;

        for (int i = 0; i < imageCount; i++)
        {
            fout << "第" << i + 1 << "幅图像的旋转向量:" << std::endl;
            fout << tvecsMat[i] << std::endl;
            //将旋转向量转换为相对应的旋转矩阵
            cv::Rodrigues(tvecsMat[i], rotationMatrix);
            fout << "第" << i + 1 << "幅图像的旋转矩阵:" << std::endl;
            fout << rotationMatrix << std::endl;
            fout << "第" << i + 1 << "幅图像的平移向量:" << std::endl;
            fout << rvecsMat[i] << std::endl;
        }
        std::cout << "保存完成" << std::endl;

        /************************************************************************
        显示定标结果
        *************************************************************************/
        cv::Mat mapx = cv::Mat(imageSize, CV_32FC1);
        cv::Mat mapy = cv::Mat(imageSize, CV_32FC1);
        cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
        std::cout << "显示矫正图像" << endl;
        for (int i = 0; i != imageCount; i++)
        {
            std::cout << "Frame #" << i + 1 << "..." << endl;
            //计算图片畸变矫正的映射矩阵mapx、mapy(不进行立体校正、立体校正需要使用双摄)
            initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, imageSize, CV_32FC1, mapx, mapy);
            //读取一张图片
            Mat imageSource = imread(filenames[i]);
            cv::resize(imageSource,imageSource,newsize,2,2,1);
            Mat newimage = imageSource.clone();
            //另一种不需要转换矩阵的方式
            //undistort(imageSource,newimage,cameraMatrix,distCoeffs);
            //进行校正
            remap(imageSource, newimage, mapx, mapy, INTER_LINEAR);
            imshow("原始图像", imageSource);
            imshow("矫正后图像", newimage);
            waitKey(30);
        }

        //释放资源
        //fin.close();
        fout.close();
        //system("pause");        
    }
}

int stereo_calibrate()
{
    int cnt=15;
    //摄像机内参数矩阵 M=[fx γ u0,0 fy v0,0 0 1]
    cv::Mat cameraMatrixl = cv::Mat(3, 3, CV_64F, Scalar::all(0));
    //摄像机的5个畸变系数k1,k2,p1,p2,k3
    cv::Mat distCoeffsl = cv::Mat(1, 5, CV_64F, Scalar::all(0));
    cv::Mat cameraMatrixr = cv::Mat(3, 3, CV_64F, Scalar::all(0));
    //摄像机的5个畸变系数k1,k2,p1,p2,k3
    cv::Mat distCoeffsr = cv::Mat(1, 5, CV_64F, Scalar::all(0));
    Mat R,T,E,F;
    std::vector<std::vector<Point2f>> imagePointsSeql;
    std::vector<std::vector<Point2f>> imagePointsSeqr;
    //保存标定板上角点的三维坐标
    std::vector<std::vector<cv::Point3f>> objectPointsl;
    std::vector<std::vector<cv::Point3f>> objectPointsr;
    string infilenamel="/home/yons/projects/stereocamera/sample/left1/shotL";
    string infilenamer="/home/yons/projects/stereocamera//sample/right1/shotR";
    string outfilenamel="/home/yons/projects/stereocamera/sample/left1/caliberation_result.txt";
    string outfilenamer="/home/yons/projects/stereocamera/sample/right1/caliberation_result.txt";
    string finalparafilename="/home/yons/projects/stereocamera//sample/final1.txt";
    calibrate(infilenamel,outfilenamel,cameraMatrixl,distCoeffsl,imagePointsSeql,objectPointsl,cnt);
    calibrate(infilenamer,outfilenamer,cameraMatrixr,distCoeffsr,imagePointsSeqr,objectPointsr,cnt);
    Mat Ml,Dl,Mr,Dr;
    cv::stereoCalibrate(objectPointsl,imagePointsSeql,imagePointsSeqr,cameraMatrixl,distCoeffsl,cameraMatrixr,distCoeffsr,Size(1280,720),R,T,E,F,CALIB_USE_INTRINSIC_GUESS,TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
    ofstream fout(finalparafilename);
    fout << "立体校正左相机内参数矩阵:" << std::endl;
    fout << cameraMatrixl << std::endl << std::endl;
    fout << "立体校正右相机内参数矩阵:" << std::endl;
    fout << cameraMatrixr << std::endl << std::endl;
    fout << "立体校正左畸变系数:" << std::endl;
    fout << distCoeffsl << std::endl << std::endl;
    fout << "立体校正右畸变系数:" << std::endl;
    fout << distCoeffsr << std::endl << std::endl;
    fout << "立体校正相机间旋转矩阵:" << std::endl;
    fout << R << std::endl << std::endl;
    fout << "立体校正相机间平移矩阵:" << std::endl;
    fout << T << std::endl << std::endl;
    fout << "立体校正本征矩阵:" << std::endl;
    fout << E << std::endl << std::endl;
    fout << "立体校正基础矩阵:" << std::endl;
    fout << F << std::endl << std::endl;
    Size imageSize(1280,720);
    Mat Rl,Rr,Pl,Pr;
    Mat mapLx,mapLy,mapRx,mapRy;
    Rect ValidROIL;                                /*ROI region*/
	Rect ValidROIR;
    Mat Q;
    Mat rectifiedframeL,rectifiedframeR;
    for (int i = 0; i != cnt; i++)
        {
            std::cout << "Frame #" << i + 1 << "..." << endl;
            //读取一张图片
            Mat rgbframeL=cv::imread("/home/yons/projects/stereocamera/sample/left1/shotL"+to_string(i)+".jpg");
            Mat rgbframeR=cv::imread("/home/yons/projects/stereocamera/sample/right1/shotR"+to_string(i)+".jpg");
            cv::resize(rgbframeL,rgbframeL,Size(1280,720),2,2,0);
            cv::resize(rgbframeR,rgbframeR,Size(1280,720),2,2,0);
            //另一种不需要转换矩阵的方式
            //undistort(imageSource,newimage,cameraMatrix,distCoeffs);
            //进行校正
            // undistort(rgbframeL,rectifiedframeL,cameraMatrixl,distCoeffsl);
            // undistort(rgbframeR,rectifiedframeR,cameraMatrixr,distCoeffsr);
            stereoRectify(cameraMatrixl,distCoeffsl,cameraMatrixr,distCoeffsr,imageSize,R,T,Rl,Rr,Pl,Pr,Q,CALIB_ZERO_DISPARITY,0,imageSize,&ValidROIL,&ValidROIR);     //input:Ml,Dl,Mr,Dr,R,T   OUTPUT:Rl,Rr,Pl,Pr,Q
	        initUndistortRectifyMap(cameraMatrixl,distCoeffsl,Rl,Ml,rgbframeL.size(),CV_32FC1,mapLx,mapLy);
	        initUndistortRectifyMap(cameraMatrixr,distCoeffsr,Rr,Mr,rgbframeL.size(),CV_32FC1,mapRx,mapRy);
            remap(rgbframeL,rectifiedframeL,mapLx,mapLy,INTER_LINEAR);
	        remap(rgbframeR,rectifiedframeR,mapRx,mapRy,INTER_LINEAR);
            cv::imshow("oril",rgbframeL);
            cv::imshow("recl",rectifiedframeL);
            cv::imshow("orir",rgbframeR);
            cv::imshow("recr",rectifiedframeR);
            cv::waitKey(0);
          
        }

    
   
   
}