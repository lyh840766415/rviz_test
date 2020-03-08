#include <opencv2/core.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>

//PCL Library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

//ROS
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

const double PI = 3.1415926535;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"Mat2show");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2> ("Mat2show", 1);
    PointCloud Cloud;
    sensor_msgs::PointCloud2 output;

    cv::Mat lidar_img = cv::imread("lidar_img.png",cv::IMREAD_UNCHANGED);
    cv::Mat show(16,4000,CV_32F,lidar_img.data);
    std::cout<<"depth = "<<show.depth()<<std::endl;
    std::cout<<show<<std::endl;

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
        }

    }
    Cloud.height = 1;
    Cloud.width =  Cloud.points.size();
    std::cout<<"Point Cloud Size = "<<Cloud.points.size()<<std::endl;
    Cloud.is_dense = false;
//    pcl::toROSMsg(Cloud, output);
//    output.header.frame_id = "odom";

    PointCloud transformed;

    ros::Rate loop_rate(1);
    int index = 0;
    while(ros::ok())
    {
        Eigen::AngleAxisd rotation_vector ( M_PI*index/10.0, Eigen::Vector3d ( 0,0,1 ) );
        Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
        tf.rotate(rotation_vector);
        tf.pretranslate ( Eigen::Vector3d (0,0,0));
        std::cout << "Transform matrix = \n" << tf.matrix() <<std::endl;

        pcl::transformPointCloud(Cloud,transformed,tf.matrix());

        pcl::toROSMsg(transformed, output);
        output.header.frame_id = "odom";
        pcl_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
        index++;
    }

    return 0;
}
