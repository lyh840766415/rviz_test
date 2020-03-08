#include <opencv2/core.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>

//PCL Library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

//ROS
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>

const double PI = 3.1415926535;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"Mat2show");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("PCL", 1);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("POSE",10);
    PointCloud Cloud;
    sensor_msgs::PointCloud2 output;

    double cosTheta[16] = { 0 };
    double sinTheta[16] = { 0 };
    for (int i = 0; i < 16; i++)
    {
        int theta = 15-2*i;
        cosTheta[i] = cos(theta * PI / 180.f);
        sinTheta[i] = sin(theta * PI / 180.f);
    }

    std::vector<double> sinAngle;
    std::vector<double> cosAngle;
    for (int i = 0; i < 4000; i++)
    {
        sinAngle.push_back(sin((0.09*i) * PI / 180.f));
        cosAngle.push_back(cos((0.09*i) * PI / 180.f));
    }

    ros::Rate loop_rate(5);
    std::string base_path = "/home/lyh/lab/data/lidar_data/png/lidar_img_";

    int index = 0;
    while(ros::ok())
    {
        Cloud.clear();
        std::string filename = base_path+std::to_string(index)+".png";
        std::cout<<filename<<std::endl;
        cv::Mat lidar_img = cv::imread(filename,cv::IMREAD_UNCHANGED);

        double x_tot = 0,y_tot = 0,z_tot = 0;
        double x_mean = 0,y_mean = 0,z_mean = 0;
        for(int i = 0;i<16;i++)
        {
            for(int j = 0;j<4000;j++)
            {
                if(lidar_img.at<float>(i,j)<10.0)
                    continue;
                PointT p;
                p.x = lidar_img.at<float>(i,j) * cosTheta[i] * sinAngle[j];
                p.y = lidar_img.at<float>(i,j) * cosTheta[i] * cosAngle[j];
                p.z = lidar_img.at<float>(i,j) * sinTheta[i];
                Cloud.points.push_back((p));
                x_tot += p.x;
                y_tot += p.y;
                z_tot += p.z;
            }
        }

        x_mean = x_tot/Cloud.points.size();
        y_mean = y_tot/Cloud.points.size();
        z_mean = z_tot/Cloud.points.size();

        double t_c_m_x = 0,t_c_m_y = 0,t_c_m_z = 0,t_c_m_norm = 0;;

        for(int i = 0;i<Cloud.points.size();i++)
        {
            double x_offset = (Cloud.points[i].x-x_mean);
            double y_offset = (Cloud.points[i].y-y_mean);
            double z_offset = (Cloud.points[i].z-z_mean);

            t_c_m_x += x_offset*x_offset*x_offset;
            t_c_m_y += y_offset*y_offset*y_offset;
            t_c_m_z += z_offset*z_offset*z_offset;
        }

        t_c_m_x = t_c_m_x/Cloud.points.size();
        t_c_m_y = t_c_m_y/Cloud.points.size();
        t_c_m_z = t_c_m_z/Cloud.points.size();

        std::cout<<"x = "<<t_c_m_x<<" , y = "<<t_c_m_y<<" , z = "<<t_c_m_z<<std::endl;
        t_c_m_norm = sqrt(t_c_m_x*t_c_m_x+t_c_m_y*t_c_m_y+t_c_m_z+t_c_m_z);
        double pitch = asin(t_c_m_z/t_c_m_norm);
        double yaw = atan2(t_c_m_y,t_c_m_x);
        std::cout<<"Pitch = "<<pitch*180.0/PI<<" , Yaw = "<<yaw*180.0/PI<<std::endl;
        Eigen::Matrix3d R_M = Eigen::AngleAxisd(yaw,Eigen::Vector3d(0,0,1)).matrix()*Eigen::AngleAxisd(pitch,Eigen::Vector3d(0,1,0)).matrix();
        Eigen::Quaterniond q = Eigen::Quaterniond(R_M);
        geometry_msgs::PoseStamped posemsg;
        posemsg.header.stamp = ros::Time::now();
        posemsg.header.frame_id = "odom";
        posemsg.pose.orientation.x = q.x();
        posemsg.pose.orientation.y = q.y();
        posemsg.pose.orientation.z = q.z();
        posemsg.pose.orientation.w = q.w();

        posemsg.pose.position.x = 0;
        posemsg.pose.position.y = 0;
        posemsg.pose.position.z = 0;


        Cloud.height = 1;
        Cloud.width =  Cloud.points.size();
        std::cout<<"Point Cloud Size = "<<Cloud.points.size()<<std::endl;
        Cloud.is_dense = false;
        pcl::toROSMsg(Cloud, output);
        output.header.frame_id = "odom";


        pcl_pub.publish(output);
        pose_pub.publish(posemsg);

        ros::spinOnce();
        loop_rate.sleep();
        index++;
    }

    return 0;
}
