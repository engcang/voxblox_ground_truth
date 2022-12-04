#ifndef TSDF_PCD_MAIN_H
#define TSDF_PCD_MAIN_H


#include "voxblox_in_use.h"
///// common headers
#include <iostream> //cout
#include <math.h> // pow
#include <chrono>
#include <vector>
#include <utility> // pair, make_pair
#include <mutex>
///// Eigen
#include <Eigen/Eigen> // whole Eigen library : Sparse(Linearalgebra) + Dense(Core+Geometry+LU+Cholesky+SVD+QR+Eigenvalues)
///// ROS
#include <ros/ros.h>
#include <std_msgs/String.h>
///// PCL
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>


using namespace std;
using namespace std::chrono; 
using namespace Eigen;
using namespace voxblox;



////////////////////////////////////////////////////////////////////////////////////////////////////
class tsdf_pcd_class
{
  public:
    ///// ros and tf
    ros::NodeHandle nh;
    ros::Subscriber m_loadmap_sub;
    shared_ptr<TsdfEsdfInUse> m_tsdfesdf_voxblox = nullptr;

    ///// functions
    // non-dji
    void loadmap_cb(const std_msgs::String::ConstPtr& msg);

    tsdf_pcd_class(const ros::NodeHandle& n_private) : nh(n_private)
    {
    	double m_voxblox_resolution;
      nh.param<double>("/voxblox_ground_truth/voxel_size", m_voxblox_resolution, 0.2);
      m_loadmap_sub = nh.subscribe<std_msgs::String>("/load_map", 5, &tsdf_pcd_class::loadmap_cb, this);
      // voxblox
      m_tsdfesdf_voxblox = make_shared<TsdfEsdfInUse>(nh, m_voxblox_resolution);
      ROS_WARN("Starting node...");
    }
};


void tsdf_pcd_class::loadmap_cb(const std_msgs::String::ConstPtr& msg)
{
	string path = msg->data;
  m_tsdfesdf_voxblox->loadMap(path);
  m_tsdfesdf_voxblox->updateMesh();	

  pcl::PointCloud<pcl::PointNormal> pcd_normals;
  m_tsdfesdf_voxblox->getPcdNormalFromMeshForPublish(pcd_normals);

  pcd_normals.width = 1;
  pcd_normals.height = pcd_normals.points.size();
  pcl::io::savePCDFileASCII<pcl::PointNormal> (path+"_pcdnormal.pcd", pcd_normals);
  ROS_WARN("pcd saved");
}





#endif