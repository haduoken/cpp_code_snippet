#define PCL_NO_PRECOMPILE
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
//#include <pcl/filters/impl/voxel_grid.hpp>
//#include <pcl/filters/impl/filter.hpp>
//#include <pcl/search/impl/search.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <iostream>
using namespace std;

struct PointXYZIT {
  PCL_ADD_POINT4D;  // XYZ + Padding
  PCL_ADD_INTENSITY;
  int type;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, type, type))

typedef PointXYZIT PointType;
typedef pcl::PointCloud<PointType> PclMap, PclScan;
typedef pcl::PointCloud<PointType>::Ptr PclMapPtr, PclScanPtr;
typedef pcl::VoxelGrid<PointType> PclVoxelGrid;
typedef pcl::KdTreeFLANN<PointType> PclKDTree;

int main(int argc, char** argv) {
  // 首先定义一个点云数据
  PclScanPtr scan(new PclScan());
  for (int i = 0; i < 1000; i++) {
    PointType pt;
    pt.x = 0.1 * i;
    pt.y = 0.1 * i;
    pt.z = 0.1 * i;
    scan->points.push_back(pt);
  }

  // 然后定义一个滤波器
  PclVoxelGrid filter;
  filter.setLeafSize(0.2, 0.2, 0.2);

  // 进行滤波
  PclScanPtr scan_ds(new PclScan());
  filter.setInputCloud(scan);
  filter.filter(*scan_ds);

  cout << "filter size to " << scan_ds->points.size() << endl;

  // 进行kdtree的测试
  PclKDTree tree;
  tree.setInputCloud(scan);
  vector<int> pointSearchInd;
  vector<float> pointSearchSqDis;
  const int search_size = 5;

  PointType search_pt;
  search_pt.x = 3;
  search_pt.y = 4;
  search_pt.z = 5;
  tree.nearestKSearch(search_pt, search_size, pointSearchInd, pointSearchSqDis);
  for (int i = 0; i < pointSearchInd.size(); i++) {
    cout << "find pt xyz: " << scan->points[pointSearchInd[i]].x << " " << scan->points[pointSearchInd[i]].y << " " << scan->points[pointSearchInd[i]].z
         << " dis is " << pointSearchSqDis[i] << endl;
  }
}