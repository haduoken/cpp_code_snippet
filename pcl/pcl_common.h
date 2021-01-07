#pragma once

#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/range_image/range_image.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <string>
using namespace std;

typedef pcl::PointXYZI PointType;

typedef pcl::PointCloud<PointType> PclMap, PclScan;
typedef pcl::PointCloud<PointType>::Ptr PclMapPtr, PclScanPtr;
typedef pcl::VoxelGrid<PointType> PclVoxelGrid;
typedef pcl::KdTreeFLANN<PointType> PclKDTree;
typedef pcl::KdTreeFLANN<PointType>::Ptr PclKDTreePtr;
typedef pcl::IterativeClosestPoint<PointType, PointType> PclICP;
typedef pcl::RadiusOutlierRemoval<PointType> PclOutlierRemove;

struct PointXYZIRPYTO {
  PCL_ADD_POINT4D
  PCL_ADD_INTENSITY;
  float roll;
  float pitch;
  float yaw;
  double time;
  double absOdom;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYTO, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(
                                                      float, yaw, yaw)(double, time, time)(double, absOdom, absOdom))

typedef PointXYZIRPYTO PointTypePose;
