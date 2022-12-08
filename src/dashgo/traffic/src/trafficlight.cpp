
#include <stdio.h>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include<geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"

#include <sstream>
#include <iostream>

#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<sys/time.h>



#define INNERSTOP	(65)
#define OUTERSTOP	(485)

using namespace cv;


CascadeClassifier cascade;
int dectedRed; //当前帧 红绿灯当检测结果 ，0 没有灯，1 红灯 2, 绿灯
bool findRed; //是否为红灯状态值
ros::Publisher pub;
VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, Size(640, 480));
//int redPos = INT_MAX;

int trafficDect(cv::Mat _src);//红绿灯检测
int trafficDect(cv::Mat _src) {
	/*
	Orange 0 - 22
	yellow 22 - 38
	Green 38 - 75
	blue 75 - 130
	violet 130 - 160
	red 160 -179
	*/
	int lowH = 0;//H通道的阈值范围
	int heightH = 255;
	int lowS = 90;//S通道的阈值范围
	int heightS = 255;
	int lowV = 90;//v通道的阈值
	int heightV = 255;

	cv::Mat imgHsv;
	cv::Mat imgHsvSplit2;
	//printf("trafficDect\n");
	cv::cvtColor(_src, imgHsv, cv::COLOR_RGB2HSV);//转换颜色空间
	std::vector<cv::Mat> imgHsvSplit;//颜色空间分量
	cv::split(imgHsv, imgHsvSplit);
	imgHsvSplit2 = imgHsvSplit[2];//提出V通道的分量
	cv::equalizeHist(imgHsvSplit[2],imgHsvSplit[2]);//对V通道进行直方图均
	cv::merge(imgHsvSplit, imgHsv);//将直方图均衡后的三个颜色通道进行混合
	cv::Mat imgThreshold;
	cv::inRange(imgHsv, cv::Scalar(lowH, lowS, lowV), cv::Scalar(heightH,heightS,heightV), imgThreshold);//以一定阈值进行二值化
	//imshow("imgThreshold",imgThreshold);
	//printf("inRange\n");
	//开操作 (去除一些噪点)  
	cv::Mat element = cv::getStructuringElement(MORPH_RECT, Size(3, 3));//获取滤波核，MORPH_RECT核形状，Size为核尺寸
	//printf("getStructuringElement\n");
	cv::morphologyEx(imgThreshold, imgThreshold, MORPH_OPEN, element);//开操作，对二值化图片进行滤波，去除二值图片中的小亮斑
	//printf("morphologyEx\n");
	std::vector<std::vector<Point> > contours(30);//contours检测到轮廓结果
	std::vector<cv::Vec4i> hei(30);//轮廓之间的递进关系。
	cv::findContours(imgThreshold, contours, hei,RETR_TREE, CHAIN_APPROX_NONE);//查找轮廓
	//printf("findContours");
	int maxConuntIdx = -1;
	double maxSize = -1;
	int sizeIdx = -1;
	double subSize = -1;
	int subIdx = -1;
	//对于所有的轮廓，如果有轮廓中有子轮廓，则认为该轮廓为红绿灯亮灯区域的轮廓，这是因为亮灯通常会导致红绿灯中间比较亮
	//没有轮廓有子轮廓，则取面积最大的轮廓为亮灯的区域。

	for (int  i = 0; i < contours.size(); i++)
	{
		if (maxSize < contourArea(contours[i]))
		{
			maxSize = contourArea(contours[i]);
			maxConuntIdx = i;
		}
		if ((hei[i][0,2] != -1) && (subSize < contourArea(contours[hei[i][0, 2]])))
		{
			subSize = contourArea(contours[hei[i][0, 2]]);
			subIdx = i;
		}
	}
	if (subIdx > -1)
	{
		maxConuntIdx = subIdx;
	}
	int sumY = 0;
	for (int j =0;j< contours[maxConuntIdx].size();j++)//取亮灯区域的轮廓的中所有像素的y坐标轴均值
	{
		sumY += contours[maxConuntIdx][j].y;
	}
	sumY = (int)(sumY / float(contours[maxConuntIdx].size()));
	//依据红灯在上方绿灯在下方，当亮灯轮廓在红绿灯图片中的下方是绿灯亮
	//	当亮灯轮廓在红绿灯图片中的上方是红灯亮
	
	if (sumY > _src.rows / 2)
	{
		return 1;
	}
	else
	{
		return 0;
	}

	return 0;
}


//使用分类器cascade对图片dstImage进行模板检测
int dectedImg(cv::Mat& dstImage,cv::CascadeClassifier cascade) {

	static bool lastDetect = false;//上一帧图片模板检测是否成功
	static Rect lastRect;//上一帧图片检测出来的矩形框
	static std::list<bool> isRed;//连续多帧检测出红灯的list
	static std::list<bool> isGreen;//连续多帧检测出绿灯的List
	static int redFrame = 0;//isRed中红灯的帧数
	static int greenFrame = 0;//isGreen中绿灯的帧数
	static int detectSize = 5;//需要判断红绿灯的帧数
	static int frameRed = 0;//整个视频流中检测出红灯的帧数
	static int frameGreen = 0;//整个视频流中检测出来绿灯的帧数
	static int frameCap = 0;//视频总帧数
	static float detectfrq = 0.9;//红绿灯判断阈值

	dectedRed = 0;
	//【3】检测  
	std::vector<cv::Rect> rect;//检测结果
	//使用分类器对cascade对图片dstImage进行检测，检测结果放在rect中，1.15检测过程中前后两次扫描窗口的比例系数，
	//3检测目标的相邻的邻接矩形的个数
	cascade.detectMultiScale(dstImage, rect, 1.15, 3, 0);//dstImage待检测图片，rect 检测结果，3 相邻两个可能结果的最小距离（可以修改）
	int rectSizeMax = INT_MAX;//多个可能结果红的最小距离
	int rectIdx = 0;//检测结果中最可能是红绿灯的id（只考虑一个红绿灯的情况）
	if (rect.size() > 0)//当检测出可能是红绿灯的个数大于0
	{
		if (rect.size() > 1)//当检测出红绿灯的个数大于1个
		{
			//假设连续两帧之间最大的红绿灯位置变化不大，当当前帧检测出红绿灯结果可能有多个时，
			//以这些可能的结果与上一帧红绿灯的距离最近的位置为最可能是红绿灯的结果
			if (lastDetect)
			{
				for (int i = 0; i < rect.size(); i++)
				{
					int dist = (rect[i].x - lastRect.x) * (rect[i].x - lastRect.x)
						+ (rect[i].y - lastRect.y) * (rect[i].y - lastRect.y);
					if (rectSizeMax > dist)
					{
						rectSizeMax = dist;
						rectIdx = i;
					}
				}
			}
			else {//如果上一帧没有检测成功，则取第0个
				rectIdx = 0;
			}
		}
		lastRect = rect[rectIdx];
		lastDetect = true;
		//在图片dstImage中画出最有可能是红绿灯的矩形框
		rectangle(dstImage, Point(rect[rectIdx].x, rect[rectIdx].y),
			Point(rect[rectIdx].x + rect[rectIdx].width, rect[rectIdx].y + rect[rectIdx].height)
			, Scalar(255, 255, 255), 2, 8, 0);

		//截取出矩形框中图片准备进行红绿灯颜色判断。
		Mat rectImage = dstImage.clone()(rect[rectIdx]);


		Mat rectImageGray;
		int lightColor = trafficDect(rectImage);//进行红绿灯判断，红灯0；绿灯1；
		//检测出红灯isRed +1;检测出绿灯isGreen +1;否则isRed和isGreen + 0
		if (!lightColor)
		{
			isRed.push_back(true);
			redFrame++;
			isGreen.push_back(false);
		}
		else
		{
			isRed.push_back(false);
			isGreen.push_back(true);
			greenFrame++;
		}
	}
	else
	{
		isGreen.push_back(false);
		isRed.push_back(false);
	}
	//如果连续多帧图片中红灯帧数大于一定阈值detectfrq，则返回2，
	//如果连续多帧图片中绿灯帧数大于一定阈值detectfrq，则返回1
	//否则检测失败，返回0

	if (isGreen.size() > detectSize)
	{
		bool isGreenFront = isGreen.front();
		greenFrame = isGreenFront ? greenFrame - 1 : greenFrame;
		isGreen.pop_front();
		if (greenFrame / float(detectSize) > detectfrq)
		{
			frameGreen++;
			dectedRed = 2;
			printf("检测到绿灯,绿灯帧数=%d,总帧数=%d\n", frameGreen, frameCap);
			if((int)rect.size() > 0){
				rectangle(dstImage, Point(rect[rectIdx].x, rect[rectIdx].y),
					Point(rect[rectIdx].x + rect[rectIdx].width, rect[rectIdx].y + rect[rectIdx].height)
					, Scalar(0, 255, 0), 2, 8, 0);
			}
		}
	}

	if (isRed.size() > detectSize)
	{
		bool isRedFront = isRed.front();
		redFrame = isRedFront ? redFrame - 1 : redFrame;
		isRed.pop_front();
		if (redFrame / float(detectSize) > detectfrq)//连续detectSize帧，红灯帧数redFrame与总帧数detectSize之比大于阈值detectfrq，则认为当前帧为红灯
		{
			frameRed++;
			if((int)rect.size() > 0){			
			    if(rect[rectIdx].y < INNERSTOP ||
				rect[rectIdx].x+ rect[rectIdx].width > OUTERSTOP){
				//printf("红灯位置  end x=%d,图片 width = %d\n",rect[rectIdx].x+ rect[rectIdx].width,dstImage.cols);
				dectedRed = 1;
			    };
			//printf("红灯位置 start x=%d,图片 width = %d\n",rect[rectIdx].x,dstImage.cols);
			printf("检测到红灯,红灯帧数=%d,总帧数=%d\n", frameRed, frameCap);

				rectangle(dstImage, Point(rect[rectIdx].x, rect[rectIdx].y),
					Point(rect[rectIdx].x + rect[rectIdx].width, rect[rectIdx].y + rect[rectIdx].height)
					, Scalar(0, 0, 255), 2, 8, 0);
			}
			
		}
	}

	frameCap++;
	return dectedRed;


}

Mat img;
double* data;
float lastCarRes= 0;
float kp = 0.45;
float kd = 0.15;
float left_cam_offset = 0.085;
float cmd_ang_z_max=-FLT_MAX;


//图片回调函数
void chatterCallbackCam(const sensor_msgs::Image::ConstPtr& msg)
{

    //ros图片格式转换为opencv片格式
    cv::Mat src(msg->height,msg->width,CV_8UC3);
    for(int i=0;i<msg->width*msg->height * 3;i++){
	src.data[i] = (uchar)msg->data[i];
    }

    cv::waitKey(10);
    cv::Mat dst;//= src.clone();
    cv::cvtColor(src,dst,cv::COLOR_BGR2RGB);//颜色空间转换
    //cv::imshow("dstsrc.jpg",dst);
    cv::waitKey(10);

    dectedRed=dectedImg(dst, cascade);//检测红绿灯

    if(dectedRed == 1){//检测出红灯，并保存红灯状态
	findRed = true;
    }else if(dectedRed == 2 && findRed){
	findRed = false;//检测出绿灯，且当前状态值为红灯，转换状态为绿灯
    }

    printf("findRed = %d\n",findRed);
    cv::waitKey(10);
    std_msgs::Bool isRed;
    if(findRed){
	isRed.data = true;		
	pub.publish(isRed);//发布红灯状态
	ROS_WARN("is red\n");
    }else{
		isRed.data = false;		
		pub.publish(isRed);
	}
}



int main(int argc, char **argv)
{
   ROS_WARN("*****START");

    findRed= false;
    //导入红绿灯分类器
    //cascade.xml需要与cascade放当实际路径一样
    bool loadxml=cascade.load("/home/eaibot/dashgo_ws/src/dashgo/traffic/src/cascade.xml");//
    if(!loadxml){
        ROS_WARN("没有正常导入cascade.xml");
        return 0;
    }
   //VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, Size(640, 480));
    printf("ros init\n");
    
    ros::init(argc,argv,"traffic");
    ros::NodeHandle n;
    int count = 0;

    pub = n.advertise<std_msgs::Bool>("/isRed", 2);//发布红绿灯状态
    //ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 100, chatterCallbackCam);//订阅图片话题	

	ros::spin();
//	ros::spinOnce();
	printf("done");
	return 0;


}




