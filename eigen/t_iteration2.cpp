#include <sophus/se3.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/core.hpp>
#include <random>
#include <vector>
#include <random>

#define Random(x) rand()%x
using namespace std;
using namespace Eigen;

int main(int argc, char *argv[]) {
  // 生成模拟数据
  vector<Eigen::Vector3d> pts_ref;
  for(int i=0;i<100;i++){
    pts_ref.push_back(Eigen::Vector3d(Random(i)/5.0,Random(i)/2.0,Random(i)/3.0));
  }

  // 定义转换
  Eigen::Quaterniond real_q;
  real_q = Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitY()) *Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitX());
  Eigen::Vector3d real_t(1,2,3);

  vector<Eigen::Vector3d> pts_cur;
  for(int i=0;i<pts_ref.size();i++){
    pts_cur.push_back(real_q*pts_ref[i]+real_t);
  }

  // 根据点对计算出变换T_ref_cur

}
