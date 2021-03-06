#include <iostream>
#include <math.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

//PCL Library
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

//g2o
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <chrono>

//ros
#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

const double PI = 3.1415926535;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

void uv2xyz(int x, int y, float d, Eigen::Vector3d& p)
{
    float alpha = 15-2*x;
    float theta = 0.09*y;
    p(0,0) = d*cos(alpha*PI/180.0f)*sin(theta*PI/180.0f);
    p(1,0) = d*cos(alpha*PI/180.0f)*cos(theta*PI/180.0f);
    p(2,0) = d*sin(alpha*PI/180.0f);
//    if(d == 0)
//    {
//        std::cout<<"d == 0, that will cause ur overflow"<<std::endl;
//    }
}

void xyz2uv(Eigen::Vector3d p,float& x, float& y, float& d)
{
    d = sqrt(p(0,0)*p(0,0)+p(1,0)*p(1,0)+p(2,0)*p(2,0));
    float theta = atan2(p(0,0),p(1,0))*180.0f/PI;
    float alpha = asin(p(2,0)/d)*180.0f/PI;
    x = (15-alpha)/2;
    y = (theta)/0.09;

//    if(std::isnan(x))
//    {
//        std::cout<<"ur overflow"<<std::endl;
//    }
}

class EdgeProjectXYZ2XYZ:public g2o::BaseUnaryEdge<3,double,g2o::VertexSE3Expmap>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    EdgeProjectXYZ2XYZ(int u,int v)
    {
        _u = u;
        _v = v;
    }
    EdgeProjectXYZ2XYZ(cv::Mat lidar1,cv::Mat lidar2)
    {
        _lidar1 = lidar1;
        _lidar2 = lidar2;
    }

    EdgeProjectXYZ2XYZ(int u,int v, cv::Mat lidar1, cv::Mat lidar2)
    {
        _u = u;
        _v = v;
        _lidar1 = lidar1;
        _lidar2 = lidar2;
    }

    virtual bool read(std::istream& is){}
    virtual bool write(std::ostream& os) const{}

    void computeError()
    {
        Eigen::Vector3d pl,pr,prr;
        const g2o::VertexSE3Expmap* v = static_cast<const g2o::VertexSE3Expmap*>(_vertices[0]);
        uv2xyz(_u,_v,_lidar1.at<float>(_u,_v),pl);
        pr = v->estimate().map((pl));
        float ur,vr,dr;
        xyz2uv(pr,ur,vr,dr);


        _error = Eigen::Vector3d(pr(1),pr(1),pr(1));
        is_coverge = false;
        non_converge_count++;
//        return;

        if(ur<0 || ur>15 || vr<0 || vr>3999)
        {
            _error = Eigen::Vector3d(pr(0),pr(1),pr(2));
            is_coverge = false;
            non_converge_count++;
            return;
        }

        uv2xyz(ur,vr,_lidar2.at<float>(ur,vr),prr);
        _error = prr - pr;
        is_coverge = true;
        converge_count++;
    }

    /*virtual void linearizeOplus()
    {
        if(!is_coverge)
        {
            _jacobianOplusXi(0,0) = 0;
            _jacobianOplusXi(0,1) = 0;
            _jacobianOplusXi(0,2) = 0;
            _jacobianOplusXi(0,3) = 0;
            _jacobianOplusXi(0,4) = 0;
            _jacobianOplusXi(0,5) = 0;


            _jacobianOplusXi(1,0) = 0;
            _jacobianOplusXi(1,1) = 0;
            _jacobianOplusXi(1,2) = 0;
            _jacobianOplusXi(1,3) = 0;
            _jacobianOplusXi(1,4) = 0;
            _jacobianOplusXi(1,5) = 0;


            _jacobianOplusXi(2,0) = 0;
            _jacobianOplusXi(2,1) = 0;
            _jacobianOplusXi(2,2) = 0;
            _jacobianOplusXi(2,3) = 0;
            _jacobianOplusXi(2,4) = 0;
            _jacobianOplusXi(2,5) = 0;
            return;
        }
        g2o::VertexSE3Expmap* v = static_cast<g2o::VertexSE3Expmap*>(_vertices[0]);
        Eigen::Vector3d pl,pr;
        uv2xyz(_u,_v,_lidar1.at<float>(_u,_v),pl);
        pr = v->estimate().map((pl));
        double x = pr[0];
        double y = pr[1];
        double z = pr[2];

        _jacobianOplusXi(0,0) = 0;
        _jacobianOplusXi(0,1) = z;
        _jacobianOplusXi(0,2) = -y;
        _jacobianOplusXi(0,3) = -1;
        _jacobianOplusXi(0,4) = 0;
        _jacobianOplusXi(0,5) = 0;


        _jacobianOplusXi(1,0) = -z;
        _jacobianOplusXi(1,1) = 0;
        _jacobianOplusXi(1,2) = x;
        _jacobianOplusXi(1,3) = 0;
        _jacobianOplusXi(1,4) = -1;
        _jacobianOplusXi(1,5) = 0;


        _jacobianOplusXi(2,0) = y;
        _jacobianOplusXi(2,1) = -x;
        _jacobianOplusXi(2,2) = 0;
        _jacobianOplusXi(2,3) = 0;
        _jacobianOplusXi(2,4) = 0;
        _jacobianOplusXi(2,5) = -1;
    }*/

public:
    static cv::Mat _lidar1,_lidar2;
    int _u,_v;
    static int converge_count;
    static int non_converge_count;
    bool is_coverge;

};

cv::Mat EdgeProjectXYZ2XYZ::_lidar1 = cv::Mat();
cv::Mat EdgeProjectXYZ2XYZ::_lidar2 = cv::Mat();
int EdgeProjectXYZ2XYZ::converge_count = 0;
int EdgeProjectXYZ2XYZ::non_converge_count = 0;
double cosTheta[16] = { 0 };
double sinTheta[16] = { 0 };
std::vector<double> sinAngle;
std::vector<double> cosAngle;

void computeAngle()
{
    for (int i = 0; i < 16; i++)
    {
        int theta = 15-2*i;
        cosTheta[i] = cos(theta * PI / 180.f);
        sinTheta[i] = sin(theta * PI / 180.f);
    }

    for (int i = 0; i < 4000; i++)
    {
        sinAngle.push_back(sin((0.09*i) * PI / 180.f));
        cosAngle.push_back(cos((0.09*i) * PI / 180.f));
    }
}


//void bundleAdjustment(cv::Mat lidar1,cv::Mat lidar2,cv::Mat& R,cv::Mat& T);

int main(int argc, char **argv)
{
    cv::Mat lidar_img_1 = cv::imread("/home/lyh/lab/data/lidar_data/png/lidar_img_0.png",cv::IMREAD_UNCHANGED);
    cv::Mat lidar_img_2 = cv::imread("/home/lyh/lab/data/lidar_data/png/lidar_img_0.png",cv::IMREAD_UNCHANGED);
    cv::RNG rng;
//    cv::imshow("lidar_img_1",lidar_img_1);
//    cv::imshow("lidar_img_2",lidar_img_2);
//    cv::waitKey(1000);

    //compute the auxiliary parameter
    computeAngle();
    ros::init(argc,argv,"match_2_scan");
    ros::NodeHandle nh;
    ros::Publisher pcl_pub1 = nh.advertise<sensor_msgs::PointCloud2> ("match_2_scan1", 1);
    ros::Publisher pcl_pub2 = nh.advertise<sensor_msgs::PointCloud2> ("match_2_scan2", 1);

    sensor_msgs::PointCloud2 output1,output2;
    PointCloud Cloud1,Cloud2;

//import pointcloud
    for(int i = 0;i<16;i++)
    {
        for(int j = 0;j<4000;j++)
        {
            if(lidar_img_1.at<float>(i,j)<10.0)
                continue;
            PointT p;
            p.x = lidar_img_1.at<float>(i,j) * cosTheta[i] * sinAngle[j];
            p.y = lidar_img_1.at<float>(i,j) * cosTheta[i] * cosAngle[j];
            p.z = lidar_img_1.at<float>(i,j) * sinTheta[i];
            Cloud1.points.push_back((p));
        }
    }

    Cloud1.height = 1;
    Cloud1.width =  Cloud1.points.size();
    std::cout<<"Point Cloud Size = "<<Cloud1.points.size()<<std::endl;
    Cloud1.is_dense = false;


    for(int i = 0;i<16;i++)
    {
        for(int j = 0;j<4000;j++)
        {
            if(lidar_img_2.at<float>(i,j)<10.0)
                continue;
            PointT p;
            p.x = lidar_img_2.at<float>(i,j) * cosTheta[i] * sinAngle[j];
            p.y = lidar_img_2.at<float>(i,j) * cosTheta[i] * cosAngle[j];
            p.z = lidar_img_2.at<float>(i,j) * sinTheta[i];
            Cloud2.points.push_back((p));
        }
    }

    Cloud2.height = 1;
    Cloud2.width =  Cloud2.points.size();
    std::cout<<"Point Cloud Size = "<<Cloud2.points.size()<<std::endl;
    Cloud2.is_dense = false;

//import pointcloud finish





    //initial R,T
    cv::Mat R = cv::Mat::eye(3,3,CV_64F);
    cv::Mat T = cv::Mat::zeros(3,1,CV_64F);

    int u = 0,v = 1000;
    float d = 100.0;
    Eigen::Vector3d p;
    uv2xyz(u,v,d,p);
    std::cout<<"u = "<<u<<" , v = "<<v<<" , d = "<<d<<std::endl;
    std::cout<<p(0,0)<<" , "<<p(1,0)<<" , "<<p(2,0)<<std::endl;

    std::cout<<"----------------------after convert------------------------"<<std::endl;
    float u1,v1,d1;
    xyz2uv(p,u1,v1,d1);
    std::cout<<"u = "<<u1<<" , v = "<<v1<<" , d = "<<d1<<std::endl;
    std::cout<<p(0,0)<<" , "<<p(1,0)<<" , "<<p(2,0)<<std::endl;



//    bundleAdjustment(lidar_img_1,lidar_img_2,R,T);
    std::cout<<"project is ready to go"<<std::endl;


    // 初始化g2o
    typedef g2o::BlockSolver< g2o::BlockSolverTraits<6,3> > Block;
    Block::LinearSolverType* linearSolver = new g2o::LinearSolverCSparse<Block::PoseMatrixType>(); // 线性方程求解器
    Block* solver_ptr = new Block ( linearSolver );     // 矩阵块求解器
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg ( solver_ptr );
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm ( solver );

    g2o::VertexSE3Expmap* pose = new g2o::VertexSE3Expmap();
    Eigen::Matrix3d R_mat;

    Eigen::Vector3d rv(rng.gaussian(0),rng.gaussian(0),rng.gaussian(1));
    Eigen::Vector3d tv(rng.gaussian(10000),rng.gaussian(10000),rng.gaussian(9000));

    rv.normalize();

    Eigen::AngleAxisd rotation_vector(rng.gaussian(0.1)*M_PI,rv);
    R_mat = rotation_vector.toRotationMatrix();
//    R_mat <<1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0;
    pose->setId (0);
    pose->setEstimate(g2o::SE3Quat(R_mat,tv));
    std::cout<<"T = "<<std::endl<<Eigen::Isometry3d(pose->estimate()).matrix()<<std::endl;

    optimizer.addVertex(pose);

//    std::cout<<"R_V = "<<rotation_vector.matrix()<<std::endl;

    EdgeProjectXYZ2XYZ* edge = new EdgeProjectXYZ2XYZ(8,1000,lidar_img_1,lidar_img_2);
    edge->setId(1);
    edge->setVertex(0,pose);
    edge->setInformation(Eigen::Matrix3d::Identity());
    optimizer.addEdge(edge);

//    EdgeProjectXYZ2XYZ* edge1 = new EdgeProjectXYZ2XYZ(6,3000);
//    edge1->setId(2);
//    edge1->setVertex(0,pose);
//    edge1->setInformation(Eigen::Matrix3d::Identity());
//    optimizer.addEdge(edge1);

    int index = 2;
    for(int i = 1;i<14;i+=1)
    {
        for(int j = 100; j<3900;j+=100)
        {
            if(lidar_img_1.at<float>(i,j)<=0)
                continue;
            if(lidar_img_2.at<float>(i,j)<=0)
                continue;
            EdgeProjectXYZ2XYZ* edge1 = new EdgeProjectXYZ2XYZ(i,j);
            edge1->setId(index);
            edge1->setVertex(0,pose);
            edge1->setInformation(Eigen::Matrix3d::Identity());
            optimizer.addEdge(edge1);
            index++;
//            std::cout<<"index = "<<index<<", i = "<<i<<", j = "<<j<<std::endl;
        }
    }

    optimizer.setVerbose ( false );
    optimizer.initializeOptimization();
    optimizer.optimize ( 100 );

    std::cout<<"T = "<<std::endl<<Eigen::Isometry3d(pose->estimate()).matrix()<<std::endl;

    Eigen::Isometry3d tf = Eigen::Isometry3d::Identity();
    tf.rotate(R_mat);
    tf.pretranslate (tv);
    std::cout << "Transform matrix = \n" << tf.matrix() <<std::endl;

//show 2 pointcloud
    PointCloud transformed1,transformed2;
    ros::Rate loop_rate(1);

    index = 0;
    while(ros::ok())
    {
//        pcl::transformPointCloud(Cloud2,transformed1,tf.matrix());
        pcl::transformPointCloud(Cloud2,transformed2,Eigen::Isometry3d(pose->estimate()).matrix());
        pcl::toROSMsg(Cloud1,output1);
        pcl::toROSMsg(transformed2,output2);

        output1.header.frame_id = "odom";
        output2.header.frame_id = "odom";
        pcl_pub1.publish(output1);
        pcl_pub2.publish(output2);
        ros::spinOnce();
        loop_rate.sleep();
        index++;
    }

    return 0;
}


/*
* 因为点云之间的匹配不是固定的迭代的过程中，匹配会发生变化，因此需要在g2o的edge中定义如何去寻找匹配。
* 需要在g2o的边中加入求导方式或者自动求导，否则无法优化。
* 是加入一条很大的边还是加入很多边。
*/
