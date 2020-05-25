#include "ros/ros.h"
#include "iostream"
#include <vector>
#include <fstream>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>


struct PointXYZIT_LYH {
  PCL_ADD_POINT4D
  uint8_t intensity;
  int32_t time_offset_us;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // make sure our new allocators are aligned
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT_LYH,
                                  (float, x, x)(float, y, y)(float, z, z)(
                                      uint8_t, intensity,
                                      intensity)(int32_t, time_offset_us, time_offset_us))


std::vector<std::string> split(std::string strtem,char a)
{
    std::vector<std::string> strvec;

    std::string::size_type pos1, pos2;
    pos2 = strtem.find(a);
    pos1 = 0;
    while (std::string::npos != pos2)
    {
        strvec.push_back(strtem.substr(pos1, pos2 - pos1));

        pos1 = pos2 + 1;
        pos2 = strtem.find(a, pos1);
    }
    strvec.push_back(strtem.substr(pos1));
    return strvec;
}


int main(int argc, char **argv)
{
    ros::init(argc,argv,"livox_2_bag");
    ros::NodeHandle nh("~");
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("livox_point_cloud", 10);

    std::string input_lidar_path = "/home/lyh/lab/data/livox_0523/2020-05-23_17-28-06.csv";
    int pts_per_packet = 10000;
    int start_time = 34947;
//    int start_time = 0;
    int end_time = 34983;
    char s[1000] = {0};


    std::cout<<"start_time = "<<start_time<<" , end_time = "<<end_time<<std::endl;

    std::ifstream tf(input_lidar_path,std::ios::in);
    std::vector<std::string> substr;
    double last_time = 0;

    std::vector<double> vx;
    std::vector<double> vy;
    std::vector<double> vz;
    std::vector<double> vt;
    vx.resize(100000);
    vy.resize(100000);
    vz.resize(100000);
    vt.resize(100000);
    int point_cnt = 0;

    while(!tf.eof())
    {
        tf.getline(s,sizeof(s));
        std::string str(s);

        substr = split(s,',');
        double cur_time = atof(substr[1].c_str());

        if(cur_time-last_time>=1e6)
        {
            last_time = cur_time;
            std::cout<<int(cur_time/1e6)<<std::endl;
        }


        if(cur_time<start_time*1e6)
            continue;
        if(cur_time>end_time*1e6)
            break;


        double x =  atof(substr[2].c_str())/1e3;
        double y =  atof(substr[3].c_str())/1e3;
        double z =  atof(substr[4].c_str())/1e3;
        if(sqrt(x*x+y*y+z*z)<1)
            continue;

        vx[point_cnt]=x;
        vy[point_cnt]=y;
        vz[point_cnt]=z;


        vt[point_cnt]=cur_time;
        point_cnt++;

        if(point_cnt>pts_per_packet)
        {
            point_cnt=0;
            pcl::PointCloud<PointXYZIT_LYH>::Ptr point_cloud(new pcl::PointCloud<PointXYZIT_LYH>);
            point_cloud->header.frame_id = "odom";
            point_cloud->height = 1;
            point_cloud->header.stamp = static_cast<uint64_t>(vt[0]);
            std::cout<< point_cloud->header.stamp <<std::endl;

            for(int i = 0;i<=pts_per_packet;i++)
            {
                PointXYZIT_LYH point;
                point.x = vx[i];
                point.y = vy[i];
                point.z = vz[i];
                point.time_offset_us = vt[i]-vt[0];
//                std::cout<< point.time_offset_us <<std::endl;
                point_cloud->points.push_back(point);
                ++point_cloud->width;
            }

            sensor_msgs::PointCloud2 pc_msg;
            pcl::toROSMsg(*point_cloud, pc_msg);
            point_cloud_pub.publish(pc_msg);
            std::cout<<"publish"<<std::endl;
        }

//        std::cout<<std::fixed<<std::setprecision(6)<<cur_time<<" "<<x<<" "<<y<<" "<<z<<std::endl;
    }

}
