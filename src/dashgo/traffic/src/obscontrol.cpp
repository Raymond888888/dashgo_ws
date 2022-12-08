#include "ros/ros.h"
#include <math.h>
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Bool.h"

#define    min_check_angle   	(-0.75)		//障碍物最小角度
#define    max_check_angle  	(0.75)		//障碍物最大角度
#define    min_check_distance   (0.1)		//障碍物最小距离
#define    max_check_distance   (1.0)		//障碍物最大角度
#define	   obsThr		(50)		//障碍物判断阈值


ros::Publisher pub;
int laser_count=0,throttle_scans=2;
//float min_check_angle,max_check_angle,min_check_distance,max_check_distance;
int obstacles_num;	//障碍物点数
//int obsThr = 50;

void scanCallback(const sensor_msgs::LaserScan msg)
{
    laser_count++;	//雷达数据帧数

    if((laser_count%throttle_scans)!=0)	//依据采样频率throttle_scans进行采样
    {
       return;
    }

    float current_point_angle,current_point_distance;

    int origin_scan_len=msg.ranges.size();
    //ROS_WARN("****origin_scan_len=%d,msg.ranges.size=%d\n",origin_scan_len,int(msg.ranges.size()));
    for(int i=0;i<origin_scan_len;i++)
    {
	//ROS_WARN("obstacles_num\n");
        current_point_angle=msg.angle_min+i*msg.angle_increment;	//获取当前点当角度值
        if((current_point_angle>=min_check_angle)&&(current_point_angle<=max_check_angle))
        {//角度满足一定条件
            current_point_distance=msg.ranges[i];	//获取当前点的距离值
            if((current_point_distance>=min_check_distance)&&(current_point_distance<=max_check_distance))
            {//距离满足一定条件
                //障碍物点数+1
                obstacles_num++;
            }
        }
        
    }

    std_msgs::Bool isRed;
    if(obstacles_num > obsThr){//障碍物点数大于一定阈值
	isRed.data = true;	//障碍物状态为true
	pub.publish(isRed);
	ROS_WARN("is obs*************num obs = %d\n",obstacles_num);
    }else{			//障碍物点数不大于阈值
	isRed.data = false;	//障碍物状态fales		
	pub.publish(isRed);
	ROS_WARN("*************not **num obs = %d\n",obstacles_num);
    }

    obstacles_num=0;
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "scan_control");
    ros::NodeHandle n;
/*
    n.param<std::float>("min_check_angle", min_check_angle, "-1.05");
    n.param<std::float>("max_check_angle", max_check_angle, "-1.05");
    
    n.param<std::float>("min_check_distance", min_check_distance, "0.1");
    n.param<std::float>("max_check_distance", max_check_distance, "1.0");
    

*/
    

    ROS_WARN("****min_check_angle=%f,max_check_angle=%f,min_check_distance=%f,\
    max_check_angle=%f",min_check_angle,max_check_angle,min_check_distance,max_check_angle);
    
    obstacles_num=0;
    
    pub = n.advertise<std_msgs::Bool>("/isObs", 2);
    ros::Subscriber origin_scan_data = n.subscribe("/scan", 1, scanCallback);

    ros::Rate loop_rate(10);	//设置障碍物状态发布频率
    ros::spin();
    return 0;
}
