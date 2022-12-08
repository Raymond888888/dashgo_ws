
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/Marker.h>
// #include <turtlebot_msgs/SetFollowState.h>

#include "dynamic_reconfigure/server.h"
//#include "turtlebot_follower/FollowerConfig.h"

#include <depth_image_proc/depth_traits.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"



ros::Publisher linear_x;
static float max_z_ = 1.5;
static float max_x_ = 0.7;
static float max_y_ = 0.5;
static float min_x_ = -0.7;
static float min_y_ = -0.05;
static float goal_z_ = 0.4;



 void imagecb(const sensor_msgs::ImageConstPtr& depth_msg)
  {

    // Precompute the sin function for each row and column预计算每行每列的正弦函数
    uint32_t image_width = depth_msg->width;
    float x_radians_per_pixel = 60.0/57.0/image_width; //每个像素的弧度
    float sin_pixel_x[image_width];
    for (int x = 0; x < image_width; ++x) {
    //求出所有像素当正弦值
      sin_pixel_x[x] = sin((x - image_width/ 2.0)  * x_radians_per_pixel);
    }

    uint32_t image_height = depth_msg->height;
    float y_radians_per_pixel = 45.0/57.0/image_width;
    float sin_pixel_y[image_height];
    for (int y = 0; y < image_height; ++y) {
      // Sign opposite x for y up values
      sin_pixel_y[y] = sin((image_height/ 2.0 - y)  * y_radians_per_pixel);
    }

    //X,Y,Z of the centroid 质心的xyz
    float x = 0.0;
    float y = 0.0;
    float z = 1e6;
    //Number of points observed 观察的点数
    unsigned int n = 0;

    //Iterate through all the points in the region and find the average of the position
    //迭代通过该区域的所有点，找到位置的平均值
    const float* depth_row = reinterpret_cast<const float*>(&depth_msg->data[0]);
    int row_step = depth_msg->step / sizeof(float);
    for (int v = 0; v < (int)depth_msg->height; ++v, depth_row += row_step)
    {
     for (int u = 0; u < (int)depth_msg->width; ++u)
     {
	//将深度转换为实际坐标
       float depth = depth_image_proc::DepthTraits<float>::toMeters(depth_row[u]);
       //不是有效的深度值或者深度超出max_z_
       if (!depth_image_proc::DepthTraits<float>::valid(depth) || depth > max_z_) continue;
       float y_val = sin_pixel_y[v] * depth;	//求解出x方向当坐标
       float x_val = sin_pixel_x[u] * depth;	//求解出y方向当坐标
       if ( y_val > min_y_ && y_val < max_y_ &&
            x_val > min_x_ && x_val < max_x_)
       {
         x += x_val;
         y += y_val;
         z = std::min(z, depth); //approximate depth as forward.找出跟踪范围内距离当最小值
         n++;
       }
     }
    }

	//ROS_WARN("min_x_=%f,min_y_=%f,max_x_=%f,max_y_=%f\n",min_x_,min_y_,max_x_,max_y_);


    //If there are points, find the centroid and calculate the command goal.
    //If there are no points, simply publish a stop goal.
    //如果有点，找到质心并计算命令目标。如果没有点，只需发布停止消息。
    if (n>100)	//被跟踪目标在跟踪范围内大于一定的阈值，开始跟踪
    {
      x /= n;
      y /= n;
      if(z > max_z_){	//跟踪距离大于最大距离，小车停车
        ROS_WARN( "Centroid too far away %f, stopping the robot", z);
//        if (enabled_)
//        {
         // cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
//	std_msgs::Bool cmd_vel_x;
//	cmd_vel_x.data =  0;
//	linear_x.publish(cmd_vel_x);
//        }
        return;
      }
	//否则进行正常跟踪
      ROS_WARN( "Centroid at %f %f %f with %d points", x, y, z, n);
//      publishMarker(x, y, z);

//      if (enabled_)
//      {
	
		std_msgs::Bool cmd_vel_x;
		if((z - goal_z_) > 0) {	//两车之间当距离大于最小距离，发布后车行走状态
			cmd_vel_x.data = true;	
		}else{
			cmd_vel_x.data = false;	//两车之间当距离小于最小距离，发布后车停车状态
		}
		linear_x.publish(cmd_vel_x);

		ROS_WARN("publish cmd\n");
//      }
    }
    else//被跟踪目标在跟踪范围内不大于一定的阈值，跟踪失败，发布小车停车状态
    {
      ROS_WARN( "Not enough points(%d) detected, stopping the robot", n);

//      if (enabled_)
//      {
        //cmdpub_.publish(geometry_msgs::TwistPtr(new geometry_msgs::Twist()));
	std_msgs::Bool cmd_vel_x;
	cmd_vel_x.data =  0;
	linear_x.publish(cmd_vel_x);
//      }
    }

//    publishBbox();
  }




int main(int argc,char** argv)
{
    ros::init(argc, argv, "turtbot_follwer");
    ros::NodeHandle n;

    ros::Subscriber sub_= n.subscribe<sensor_msgs::Image>("/camera/depth/image_rect", 1, imagecb);	//深度图片订阅话题
    
    linear_x = n.advertise<std_msgs::Bool> ("cmd_vel_x", 1);		//跟踪行走与停止状态发布话题

    ros::Rate loop_rate(10);	//设置障碍物状态发布频率
    ros::spin();
    return 0;
}
