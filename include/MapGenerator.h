#ifndef MAPGENERATOR_H
#define MAPGENERATOR_H

#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Geometry> 
#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl/octree/octree.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_iterator.h>
#include <pcl/octree/octree_impl.h> 

#include <pcl/correspondence.h>
#include <pcl/common/transforms.h>
#include <pcl_ros/transforms.h> 

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include "MapPublisher.h"


using namespace std;
using namespace Eigen;


typedef struct _image_velo_correspondence image_velo_correspondence;
struct _image_velo_correspondence {
    vector<int> i_idx;
    vector<int> v_idx;
    vector<float> info;
};

class MapGenerator
{
public:
    MapGenerator():
    resolution(1.0f), Nmin(10), Nsigma(3.0f), tmin(0.1f), tmax(0.5f), max_iter(50),
    velo_cloud(new pcl::PointCloud<pcl::PointXYZ>),
    merged_cloud(new pcl::PointCloud<pcl::PointXYZ>),
    Velo_received(false), ICP_start(false), Pose_started(false)
    {
        //Set Subscriber
        sub_veloptcloud = nh.subscribe("/kitti/velodyne_points", 1, &MapGenerator::VeloPtsCallback, this); 
    }

    ~MapGenerator(){
    //save velodyne point clouds
    pcl::io::savePLYFileASCII ("merged_icp3.ply", *merged_cloud);
    }

    //publish
    void Refresh();

private:
    //for ros subscription
    ros::NodeHandle nh;
    ros::Subscriber sub_veloptcloud;
    ros::Publisher publisher;
    tf::TransformListener tlistener;
    ros::Time velo_time;

    //for broadcast    
    tf::TransformBroadcaster mTfBr;

    //input data
    pcl::PointCloud<pcl::PointXYZ>::Ptr velo_cloud;
    tf::StampedTransform wtb;
    tf::StampedTransform ctv;
    Matrix4f ODO_pose;
    Matrix4f ODO_raw;
    Matrix4f EST_pose;
    Matrix4f cTv;
    bool Pose_started;

    //output
    pcl::PointCloud<pcl::PointXYZ>::Ptr merged_cloud;

    //Callbacks
    void VeloPtsCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
    bool Velo_received; 

    //MapPublisher
    MapPublisher MapPub;

    //ICP
    void VCICP(pcl::PointCloud<pcl::PointXYZ>::Ptr& v_cloud,
               pcl::PointCloud<pcl::PointXYZ>::Ptr& c_cloud);   
    void CorrespondenceUpdate(const pcl::PointCloud<pcl::PointXYZ>::Ptr& v_cloud,
                              const pcl::PointCloud<pcl::PointXYZ>::Ptr& c_cloud,
                              const int iter,
                              pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> LPDoctree);
    Matrix4f Optimization(const pcl::PointCloud<pcl::PointXYZ>::Ptr& v_cloud,
                          const pcl::PointCloud<pcl::PointXYZ>::Ptr& c_cloud,
                          float th2); 
    
    Matrix4f SE3toMat(const g2o::SE3Quat &SE3)
    {
    Eigen::Matrix3f eigR = SE3.rotation().toRotationMatrix().cast <float> ();
    Eigen::Vector3f eigt = SE3.translation().cast <float> ();
    Matrix4f T = Matrix4f::Identity();
    T.block<3,3>(0,0) = eigR;
    T(0,3) = eigt[0];
    T(1,3) = eigt[1];
    T(2,3) = eigt[2];
    return T;
    }

    Matrix4f Sim3toMat(const g2o::Sim3 &Sim3)
    {
    Eigen::Matrix3f eigR = Sim3.rotation().toRotationMatrix().cast <float> ();
    Eigen::Vector3f eigt = Sim3.translation().cast <float> ();
    float s = (float) Sim3.scale();
    Matrix4f T = Matrix4f::Identity();
    T.block<3,3>(0,0) = s*eigR;
    T(0,3) = eigt[0];
    T(1,3) = eigt[1];
    T(2,3) = eigt[2];
    return T;
    }

    //ICP parameters
    float tmax;
    float tmin;
    int Nmin;
    float Nsigma;
    float resolution;
    size_t max_iter;
    bool ICP_start;
    //g2o::Sim3 g2oS;
    image_velo_correspondence iv_corr;

};


#endif // MAPGENERATOR_H
