#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"

#include<geometry_msgs/Twist.h>
#include "sensor_msgs/Image.h"

#define MIN_LANE	(20)	//车道线最小宽度
#define MAX_LANE	(40)	//车道线最大宽度
#define STRIDE_GAP	(6)	//车道线宽度阈值
#define TH_GD		(1)	//车道线截面梯度最小值
#define GD_BIN		(100)	//认为存在梯度的最小值

#define IMG_SHOW	(0)	//调试图片显示开关
#define CAR_CENTER      (100)	//小车中心像素坐标
#define LINEAR_X	(0.25)	//小车行走线速度
#define KP		(0.6)	//小车行走当比列系数
#define KD		(0.15)	//小车行走当比列系数
#define LOSELANE	(10)	//小车连续车道线识别失败帧数


#ifndef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#endif

#ifndef min
#define min(a, b) ((a) < (b) ? (a) : (b))
#endif

bool ISRED=false;	//红绿灯状态
bool ISOBS=false;	//障碍物状态

typedef struct
{
	int st;
	int ed;
	int y;
}ST_line;

using namespace cv;



static int m_frames = 0;
static int m_bType = 0; //0±íÊŸ¿ÉÄÜÍâ»·ºÍÄÚ»·¶ŒÓÐ¿ÉÄÜ£¬1±íÊŸÍâ»·£¬2±íÊŸÄÚ»·
static Mat m_frHLS, m_HLS[3];
static Mat m_frGradL, m_frGradS, m_frBinL, m_frBinS;
static ST_line m_lineS[2048], m_lineL[2048];
static int m_nLnS = 0;
static int m_nLnL = 0;
static int m_histS[640], m_histL[640];

static void cellPickdUp();
static int analysis();


//HLS,L¿ÉÊ¶±ð°×É«£¬S¿ÉÊ¶±ð»ÆÉ«
//车道线识别过程
//输入：frame zed相机传过来待处理图片
//输出： 车道线像素坐标
int TrafficLineAnalysis(Mat frame)
{
	//Mat frame = frIn(Rect(0, (frIn.rows >> 1) - 1, frIn.cols, (frIn.rows >> 1)));

	cvtColor(frame, m_frHLS, cv::COLOR_RGB2HLS);//转换颜色空间
	//GaussianBlur(frGray, frGray, Size(3, 3), 0, 0, BORDER_DEFAULT);

	split(m_frHLS, m_HLS);//HLS颜色空间通道分离

	Sobel(m_HLS[1], m_frGradL, m_HLS[1].depth(), 1, 0);//求解L通道打梯度值
	Sobel(m_HLS[2], m_frGradS, m_HLS[2].depth(), 1, 0);//求解S通道打梯度值


	threshold(m_HLS[1], m_frBinL, 140, 200, cv::THRESH_BINARY);//对L通道进行二值化
	threshold(m_HLS[2], m_frBinS, 140, 200, cv::THRESH_BINARY);//对S通道进行二值化
/*
#ifdef IMG_SHOW
	imshow("frHLS-frL", m_HLS[1]);
	imshow("frHLS-frS", m_HLS[2]);
	imshow("frHLS-L-GRAD", m_frGradL);
	imshow("frHLS-S-GRAD", m_frGradS);
	imshow("frHLS-L-BIN", m_frBinL);
	imshow("frHLS-S-BIN", m_frBinS);
#endif
*/

	cellPickdUp();//车道线截面提取



	return analysis();//车道线坐标确定
}



//车道线截面提取
static void cellPickdUp()
{
	Mat matTmp;
	Mat element = getStructuringElement(MORPH_RECT, Size(3, 3));	//获取滤波核
	int i, j;
	int bHold, k, v, v_gd, st, ed;

	/////////////////////////////////////S通道处理
	if (0 == m_bType || 2 == m_bType)
	{
		dilate(m_frBinS, matTmp, element);//S通道二值图像膨胀
		erode(matTmp, m_frBinS, element);//S通道二值图像腐蚀
		erode(m_frBinS, matTmp, element);
		dilate(matTmp, m_frBinS, element);

		m_nLnS = 0;			//车道线截面当个数

		//Mat roi = m_frBinS(Rect(0, (m_frBinS.rows >> 3), (m_frBinS.cols >> 1), ))

		//选取一个适当大小当区域，实验中区域为原图片左下角的0.5倍宽和0.2倍高的大小
		for (i = (m_frBinS.rows*0.8); i < m_frBinS.rows; i++)
		{
			bHold = st = ed = v_gd = 0;//临时变量初始化
			for (j = 0; j < (m_frBinS.cols >> 1); j++)
			{
				v = m_frBinS.at<uchar>(i, j);//获取S通道的像素值

				if (0 == bHold)
				{
					//找到s通道中第一个像素值不为0的点，作为截面当开始坐标。
					if (0 != v)
					{
						k = 1;	//S通道中像素值不为0当点当个数
						bHold = 1;
						st = j;
						ed = 0;
						v_gd = m_frGradS.at<uchar>(i, j) > GD_BIN ? 1 : 0;//梯度值个数
					}
				}
				else
				{
					if (0 != v)
					{
						k++;
						v_gd += (m_frGradS.at<uchar>(i, j) > GD_BIN ? 1 : 0);
					}
					else
					{
						ed = j - 1;
						//当一条线段存在的长度在一定范围以内，且存在一定当梯度时，认为这条线段为车道线截面
						if (ed - st + 1 >= MIN_LANE && ed - st + 1 <= MAX_LANE && v_gd >= TH_GD)
						{
							m_lineS[m_nLnS].st = st;//车道线截面的开始坐标
							m_lineS[m_nLnS].ed = ed;//车道线截面的结束坐标
							m_lineS[m_nLnS].y = i;//车道线当y坐标
							m_nLnS++;//车道线截面个数+1
						}

						bHold = 0;
					}
				}
			}
			//车道线位于图片中线附近
			if (0 != bHold && 0 != k)
			{
				ed = j;

				if (ed - st + 1 >= MIN_LANE && ed - st + 1 <= MAX_LANE && v_gd >= TH_GD)
				{
					m_lineS[m_nLnS].st = st;
					m_lineS[m_nLnS].ed = ed;
					m_lineS[m_nLnS].y = i;
					m_nLnS++;
				}

				bHold = 0;
			}
		}

	}


	/////////////////////////////L和处理过程和S通道处理过程一样
	if (0 == m_bType || 1 == m_bType)
	{
		m_nLnL = 0;
		dilate(m_frBinL, matTmp, element);
		erode(matTmp, m_frBinL, element);
		erode(m_frBinL, matTmp, element);
		dilate(matTmp, m_frBinL, element);

		//for (i = m_frBinL.rows - 1; i > (m_frBinL.rows >> 3); i--)
		for (i = (m_frBinL.rows*0.8); i < m_frBinL.rows; i++)
		{
			bHold = st = ed = v_gd = 0;
			for (j = 0; j < (m_frBinL.cols >> 1); j++)
			{
				v = m_frBinL.at<uchar>(i, j);

				if (0 == bHold)
				{
					if (0 != v)
					{
						k = 1;
						bHold = 1;
						st = j;
						ed = 0;
						int vv = m_frGradL.at<uchar>(i, j);

						if (vv > 100)
						{
							int ff = 1;
						}

						v_gd = m_frGradL.at<uchar>(i, j) > GD_BIN ? 1 : 0;
					}
				}
				else
				{
					if (0 != v)
					{
						k++;
						v_gd += (m_frGradL.at<uchar>(i, j) > GD_BIN ? 1 : 0);
					}
					else
					{
						ed = j - 1;

						if (ed - st + 1 >= MIN_LANE && ed - st + 1 <= MAX_LANE && v_gd >= TH_GD)
						{
							m_lineL[m_nLnL].st = st;
							m_lineL[m_nLnL].ed = ed;
							m_lineL[m_nLnL].y = i;
							m_nLnL++;
						}

						bHold = 0;
					}
				}
			}

			if (0 != bHold && 0 != k)
			{
				ed = j;

				if (ed - st + 1 >= MIN_LANE && ed - st + 1 <= MAX_LANE && v_gd >= TH_GD)
				{
					m_lineL[m_nLnL].st = st;
					m_lineL[m_nLnL].ed = ed;
					m_lineL[m_nLnL].y = i;
					m_nLnL++;
				}

				bHold = 0;
			}
		}
	}

	///////////////////////////////////////


	return;
}



static int analysis()
{
	int i, j;
	int k, l, v;
	int	pk_s, pkV_s;
	int pk_l, pkV_l;

	pk_s = pk_l = -1;
	pkV_s = pkV_l = -1;
	if (0 == m_bType || 2 == m_bType)
	{
		memset(m_histS, 0, sizeof(m_histS));
		for (i = 0; i < m_nLnS; i++)
		{
			m_histS[(m_lineS[i].ed + m_lineS[i].st >> 1)]++;//统计车道线截面线段中点当个数
		}


		for (i = 0; i < (m_frHLS.cols * 3 >> 2); i++)
		{
			l = 0;
			
			for (j = i; j < i + STRIDE_GAP; j++)
			{
				l += m_histS[j];//在一定宽度中累计出车道线中点的个数
			}
			//找出车道线中点的数量最大的位置
			if (pkV_s < l && l > 0)
			{
				pkV_s = l;
				pk_s = i;
			}
		}
	}

	if (0 == m_bType || 1 == m_bType)
	{
		memset(m_histL, 0, sizeof(m_histL));
		for (i = 0; i < m_nLnL; i++)
		{
			m_histL[(m_lineL[i].ed + m_lineL[i].st >> 1)]++;
		}
		pk_l = -1;
		pkV_l = -1;
		for (i = 0; i < (m_frHLS.cols * 3 >> 2); i++)
		{
			l = 0;
			for (j = i; j < i + STRIDE_GAP; j++)
			{
				l += m_histL[j];
			}

			if (pkV_l < l && l > 0)
			{
				pkV_l = l;
				pk_l = i;
			}
		}
	}

	if (0 == m_bType)//选取车道线坐标的最优解
	{
		//选取车道线截面数量较多的通道当车道线坐标为最终当坐标。
		if (abs(pkV_l - pkV_s) >= 5)
		{
			m_bType = pkV_s > pkV_l ? 2 : 1;
		}
	}


	//printf("pkv:%d \n", max(pkV_s, pkV_l));

	return 2 == m_bType ? pk_s : (1 == m_bType ? pk_l : -1);
}


bool findFollower = false;
float distance_z = 0;

void followerCallback(const std_msgs::Bool _msg){

	ROS_WARN("follower call back linear x = %d\n",_msg.data);
	distance_z = _msg.data;
	findFollower = true;


}


float lastCarRes= 0;
int main(int argc, char **argv)
{
	VideoCapture capture;
	capture.open(0);//打开zed相机

	
	ROS_WARN("*****START");
	 ros::init(argc,argv,"trafficLaneTrack");//初始化ROS节点
         ros::NodeHandle n;
         int count = 0;
        ros::Rate loop_rate(10);//定义速度发布频率
        ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 5);//定义速度发布器
        ros::Subscriber sub_follower = n.subscribe("cmd_vel_x", 2, followerCallback);//订阅跟踪状态
	//VideoCapture capture("/home/eaibot/dashgo_ws/zed4.avi");

	if (!capture.isOpened())
	{
		printf("摄像头没有正常打开，重新插拔工控机上当摄像头\n");
		return 0;
	}
	waitKey(2000);
	Mat frame;//当前帧图片
	int nFrames = 0;//图片帧数
	int frameWidth = capture.get(CAP_PROP_FRAME_WIDTH);//图片宽
	int frameHeight = capture.get(CAP_PROP_FRAME_HEIGHT);//图片高
	VideoWriter video("outcpp.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, Size(frameWidth, frameHeight));
	int loseLane = 0;//车道线失败帧数
	while (ros::ok())
	{
		capture.read(frame);
		if(frame.empty())
		{
			break;
		}
		video.write(frame);
		nFrames++;
		Mat frIn = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));//截取zed的左目图片
		int pos = TrafficLineAnalysis(frIn);//车道线识别
		float carRes = 0;//车道线当实际坐标
		if (pos >= 0)//车道线识别成功
		{
			circle(frIn, Point(pos, frIn.rows - 1), 20, CV_RGB(0, 255, 0), 4);
			carRes = (pos - CAR_CENTER) * 3.7/911;//车道线坐标校正过后转换到实际距离（以m为单位）
			loseLane = 0;//车道线识别失败帧数=0
		}else{//车道线识别失败
			carRes = lastCarRes;//车道线坐标为上一个时刻当坐标
			loseLane++;//车道线识别失败帧数+1
		}
		
		geometry_msgs::Twist cmd_red;
		if( loseLane >LOSELANE){//有红绿灯或者障碍物或者连续5帧车道线检测失败，停车
			cmd_red.linear.x = 0.0;
			cmd_red.linear.y = 0;
			cmd_red.linear.z = 0;
			cmd_red.angular.x = 0;
			cmd_red.angular.y = 0;
			cmd_red.angular.z = 0;
	
			pub.publish(cmd_red);
			ROS_WARN(" car stop ISRED = %d,ISOBS = %d,loseLane = %d\n",ISRED,ISOBS,loseLane);
	
		}else{//否则小车正常行走

			if(findFollower){//当小车处于跟踪状态时，小车与目标距离近则正常行走，否则停车
			    if(distance_z){
				cmd_red.linear.x = LINEAR_X;
				//findFollower = false;
				cmd_red.linear.y = 0;
				cmd_red.linear.z = 0;
				cmd_red.angular.x = 0;
				cmd_red.angular.y = 0;
				cmd_red.angular.z = -( KP *  carRes + KD * (carRes - lastCarRes));
	
				pub.publish(cmd_red);
				ROS_WARN("TRACK SUCESS pub linear.x = %f,angular.z = %f\n",cmd_red.linear.x,cmd_red.angular.z);
			    }else{
				cmd_red.linear.x = 0.00;
				//cmd_red.linear.x = LINEAR_X;
				cmd_red.linear.y = 0;
				cmd_red.linear.z = 0;
				cmd_red.angular.x = 0;
				cmd_red.angular.y = 0;
				cmd_red.angular.z = 0;
	
				pub.publish(cmd_red);
				ROS_WARN("TRACK FAILURE pub linear.x = %f,angular.z = %f\n",cmd_red.linear.x,cmd_red.angular.z);
			    }

			}else{//小车没有处于跟踪状态下
				//cmd_red.linear.x = 0.00;
				cmd_red.linear.x = LINEAR_X;
				cmd_red.linear.y = 0;
				cmd_red.linear.z = 0;
				cmd_red.angular.x = 0;
				cmd_red.angular.y = 0;
				cmd_red.angular.z = -( KP *  carRes + KD * (carRes - lastCarRes));
	
				pub.publish(cmd_red);
				ROS_WARN("NOT TRACK  pub linear.x = %f,angular.z = %f\n",cmd_red.linear.x,cmd_red.angular.z);
			}

	
		}
		

		lastCarRes = carRes;//更新上一个时刻的小车位置

		//imshow("frame", frame);
		//imshow("frIn", frIn);
		
		printf("nframes:%d,   pos:%d!!!!!!!!!!!\n", nFrames, pos);
		ros::spinOnce();
		loop_rate.sleep();
		waitKey(5);

	}


	return 0;
}


