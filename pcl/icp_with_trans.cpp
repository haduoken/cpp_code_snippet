#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <iostream>

int main(int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>(5, 1));
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>(5, 1));

  // Fill in the CloudIn data, T1
  Eigen::Affine3f T1;
  T1 = pcl::getTransformation(1, 2, 3, 0.1, 0.2, 0.3);

  Eigen::Affine3f T2;
  T2 = pcl::getTransformation(1, 2, 3, 0.2, 0.1, 0.9);

  for (int i = 0; i < 5; i++) {
    Eigen::Vector3f origin_point, after_trans_t1, after_trans_t2;
    origin_point.x() = 1024 * rand() / (RAND_MAX + 1.0f);
    origin_point.y() = 1024 * rand() / (RAND_MAX + 1.0f);
    origin_point.z() = 1024 * rand() / (RAND_MAX + 1.0f);
    after_trans_t1 = T1 * origin_point;
    after_trans_t2 = T2 * origin_point;
    cloud_in->points[i].x = after_trans_t1.x();
    cloud_in->points[i].y = after_trans_t1.y();
    cloud_in->points[i].z = after_trans_t1.z();
    cloud_out->points[i].x = after_trans_t2.x();
    cloud_out->points[i].y = after_trans_t2.y();
    cloud_out->points[i].z = after_trans_t2.z();
  }

  std::cout << "Saved " << cloud_in->points.size() << " data points to input:" << std::endl;

  for (auto& point : *cloud_in) std::cout << point << std::endl;

  std::cout << "Transformed " << cloud_in->points.size() << " data points:" << std::endl;

  for (auto& point : *cloud_out) std::cout << point << std::endl;

  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(cloud_in);
  icp.setInputTarget(cloud_out);

  pcl::PointCloud<pcl::PointXYZ> Final;
  icp.align(Final);

  std::cout << "has converged:" << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
  std::cout << icp.getFinalTransformation() << std::endl;

  Eigen::Affine3f T1_T2, T2_T1, T2_T1_inverse,T1_T2_inverse;
  T1_T2 = T1.inverse() * T2;
  T2_T1 = T2.inverse() * T1;
  T2_T1_inverse = T2 * T1.inverse();
  T1_T2_inverse = T1 * T2.inverse();
  std::cout << "t1.inverse()*t2 is " << T1_T2.matrix() << std::endl;
  std::cout << "t2.inverse()*t1 is " << T2_T1.matrix() << std::endl;
  std::cout << "t2*t1.inverse() is " << T2_T1_inverse.matrix() << std::endl;
  std::cout << "t1*t2.inverse() is " << T1_T2_inverse.matrix() << std::endl;

  return (0);
}