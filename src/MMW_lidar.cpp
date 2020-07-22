/****************************
 * 文件名：MMW_lidar.cpp
 * 创建人：东北大学-王希哲
 * 描 述：东北大学无人驾驶实验室、毫米波雷达、标记线
 * 日 期：2020-4-10
 * 版 本：1.0.0
 ***************************/
#include "stdlib.h" 
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <iostream>
#include <pcl/filters/passthrough.h>

#include <math.h>
#include "std_msgs/String.h"            //ros定义的String数据类型
#include <stdlib.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <string>
using namespace std;

class MMW_lidar
{
public:
    MMW_lidar()
    {
        sub_vel = nh.subscribe("/gps/vel", 1, &MMW_lidar::velcallback, this);
        sub_odom = nh.subscribe("/gps/odom", 1, &MMW_lidar::odomcallback, this);

        pcl_axis = nh.advertise<sensor_msgs::PointCloud2>("/axis", 10); 
        pcl_mark = nh.advertise<sensor_msgs::PointCloud2>("/mark_line", 10);
    }
    void odomcallback(const nav_msgs::Odometry &msg);
    void velcallback(const geometry_msgs::TwistWithCovarianceStamped &msg_vel);
    
private:
  ros::NodeHandle nh;
  ros::Subscriber sub_vel, sub_odom;
  ros::Publisher pcl_axis, pcl_mark;
  std_msgs::Header point_cloud_header_;
};


void MMW_lidar::velcallback(const geometry_msgs::TwistWithCovarianceStamped &msg_vel)
{
    float vel_E=msg_vel.twist.twist.linear.x;
    float vel_N=msg_vel.twist.twist.linear.y;
    float VEL=sqrt(vel_E*vel_E+vel_N*vel_N);

    pcl::PointCloud<pcl::PointXYZI>::Ptr axis(new pcl::PointCloud<pcl::PointXYZI>);
    for (int i=0; i<500; i++)  
    {
       pcl::PointXYZI x_axis;   
       x_axis.y=0;       
       x_axis.x=i*0.01;  
       axis->points.push_back(x_axis); 
    }
    for (int i=-500; i<500; i++)  
    {
       pcl::PointXYZI y_axis;   
       y_axis.x=0;   
       y_axis.y=i*0.01;  
       axis->points.push_back(y_axis);   
    }
    for (int i=480; i<500; i++)   
    {
       pcl::PointXYZI x_axis_arrow_1;   
       x_axis_arrow_1.x=i*0.01;  
       x_axis_arrow_1.y=-0.5*x_axis_arrow_1.x+2.5;  
       axis->points.push_back(x_axis_arrow_1);  
       pcl::PointXYZI x_axis_arrow_2;  
       x_axis_arrow_2.x=i*0.01;
       x_axis_arrow_2.y=0.5*x_axis_arrow_2.x-2.5;  
       axis->points.push_back(x_axis_arrow_2);  
    }
    for (int i=480; i<500; i++)   
    {
        pcl::PointXYZI y_axis_arrow_1;  
        y_axis_arrow_1.y=i*0.01;
        y_axis_arrow_1.x=(y_axis_arrow_1.y-5)/-2; 
        axis->points.push_back(y_axis_arrow_1);  
        pcl::PointXYZI y_axis_arrow_2; 
        y_axis_arrow_2.y=i*0.01;
        y_axis_arrow_2.x=(y_axis_arrow_2.y-5)/2;  
        axis->points.push_back(y_axis_arrow_2);  
    }

    sensor_msgs::PointCloud2 axis_temp;
    pcl::toROSMsg(*axis, axis_temp); 
    axis_temp.header=point_cloud_header_ ;  
    axis_temp.header.frame_id = "/velodyne"; 
    pcl_axis.publish(axis_temp); 

    std::vector<double> vec_radius;
    pcl::PointXYZI  mark_line_points;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mark_line(new pcl::PointCloud<pcl::PointXYZI>);
    vec_radius={0.5, 1.0, 1.5, 2.0, 2.5, 3.0};
    int range=M_PI*100;
    for (int j=0; j<vec_radius.size(); j++)
    {
        for(int i=0; i<range; i++)
        {
           float theta=(-1*M_PI)/2+0.01*i;
           mark_line_points.x=vec_radius[j]*cos(theta);
           mark_line_points.y=vec_radius[j]*sin(theta);
           mark_line->points.push_back(mark_line_points);
        }
    }
    sensor_msgs::PointCloud2 mark_line_temp;
    pcl::toROSMsg(*mark_line, mark_line_temp);
    mark_line_temp.header =point_cloud_header_ ;
    mark_line_temp.header.frame_id = "/velodyne";
    pcl_mark.publish(mark_line_temp);

}
void MMW_lidar::odomcallback(const nav_msgs::Odometry &msg)
{
     tf::Quaternion quat;
     double roll, pitch, yaw;
     tf::quaternionMsgToTF(msg.pose.pose.orientation , quat);
     tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "MMW_lidar");
  MMW_lidar start_detec;
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
}
