#include <sophus/se3.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <opencv2/core.hpp>
#include <random>
#include <vector>

using namespace std;
using namespace Eigen;

vector<Matrix<double, 1, 6>> jacobians;
vector<double> fs;

Sophus::SE3 trans;

vector<Vector3d> pts;

float LM_param_ = 1;
const float minGradient = 1e-4;

bool LMOptimization() {
  int dimension = pts.size();
  cv::Mat matA(dimension, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat matAt(6, dimension, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
  cv::Mat matC = cv::Mat::eye(6, 6, CV_32F);
  matC = LM_param_ * matC;
  cv::Mat matB(dimension, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
  cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

  Eigen::Vector3d point_ori;
  vector<double> coeff;
  float totle_dis = 0;
  for (int i = 0; i < dimension; ++i) {
    //    coeff = coeffs[i];
    //    Eigen::RowVector4d c_coeff(coeff[0], coeff[1], coeff[2], 1.0);
    //    float d2 = coeff[3];
    totle_dis += fs[i];
    //
    //    point_ori.x() = pts[i](0);
    //    point_ori.y() = pts[i](1);
    //    point_ori.z() = pts[i](2);
    //
    //    Eigen::Matrix<double, 4, 6> lie_algebra_jacobin = Eigen::MatrixXd::Zero(4, 6);
    //    lie_algebra_jacobin.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
    //    lie_algebra_jacobin.block<3, 3>(0, 3) = -Sophus::SO3::hat(point_ori);
    //    Eigen::Matrix<double, 1, 6> line_jacobin = c_coeff * lie_algebra_jacobin;

    matA.at<float>(i, 0) = jacobians[i](0, 0);
    matA.at<float>(i, 1) = jacobians[i](0, 1);
    matA.at<float>(i, 2) = jacobians[i](0, 2);
    matA.at<float>(i, 3) = jacobians[i](0, 3);
    matA.at<float>(i, 4) = jacobians[i](0, 4);
    matA.at<float>(i, 5) = jacobians[i](0, 5);
    matB.at<float>(i, 0) = -0.02 * fs[i];
    //        std::cout<<"jacobin : "<<arx<<", "<<ary<<", "<<arz<<", "<<atx<<",
    //        "<<aty<<", "<<atz<<", "<<d2<<std::endl;
  }
  totle_dis /= dimension;
  //    std::cout<<"totle dis : "<<totle_dis<<std::endl;
  //    if (totle_dis < totole_loss_) totole_loss_ = totle_dis;
  //    else return true;

  cv::transpose(matA, matAt);
  matAtA = matAt * matA;
  matAtB = matAt * matB;
  auto matLM = matAtA + matC;
  cv::solve(matLM, matAtB, matX, cv::DECOMP_QR);

  Sophus::SE3 update(Sophus::SO3(matX.at<float>(0, 0), matX.at<float>(1, 0), matX.at<float>(2, 0)),
                     Eigen::Vector3d(matX.at<float>(3, 0), matX.at<float>(4, 0), matX.at<float>(5, 0)));
  trans = trans * update;

  //    std::cout<<"revise : "<<matX.at<float>(0, 0)
  //            <<", "<<matX.at<float>(1, 0)
  //           <<", "<<matX.at<float>(2, 0)
  //          <<", "<<matX.at<float>(3, 0)
  //         <<", "<<matX.at<float>(4, 0)
  //        <<", "<<matX.at<float>(5, 0)<<std::endl;
  std::cout << "current trans is " << trans << "total dis" << totle_dis << std::endl;

  if (fabs(matX.at<float>(0, 0)) < minGradient && fabs(matX.at<float>(1, 0)) < minGradient && fabs(matX.at<float>(2, 0)) < minGradient &&
      fabs(matX.at<float>(3, 0)) < minGradient && fabs(matX.at<float>(4, 0)) < minGradient && fabs(matX.at<float>(5, 0)) < minGradient)
    return true;

  return false;
}

bool AddCornerConstraint(Vector3d &p0, Vector3d &p1, Vector3d &p2, int itorCount) {
  Eigen::Vector3d v12 = p2 - p1;
  Eigen::Vector3d v10 = p0 - p1;
  // 向量的2范数
  double v12_sq_norm = v12.squaredNorm();

  // 计算出距离的平方
  Eigen::Vector3d v12_10 = v12.cross(v10);
  double distance = v12_10.squaredNorm() / v12_sq_norm;

  double c = 2 / v12_sq_norm;
  Eigen::RowVector3d coeff_matrix = v12_10.transpose() * Sophus::SO3::hat(v12);
  coeff_matrix *= c;

  // 计算关于旋转量的雅各比
  auto R = trans.rotation_matrix();
  auto B = Sophus::SO3::hat(v12) * Sophus::SO3::hat(v12);
  Vector3d C = v10;
  auto A = v12_sq_norm;
  auto p_hat = Sophus::SO3::hat(p0);
  auto rtbc = R.transpose() * B * C;
  Eigen::RowVector3d j_r = (C.transpose() * B * R * p_hat - p0.transpose() * Sophus::SO3::hat(rtbc)) / A;

  // 计算关于平移量的雅各比
  auto D = (B * v10).transpose();
  auto E = v10.transpose() * B;
  Eigen::RowVector3d j_t = -(D + E) / v12_sq_norm;

  Matrix<double, 1, 6> jaco;
  jaco.block<1, 3>(0, 0) = j_r;
  jaco.block<1, 3>(0, 3) = j_t;
  jacobians.push_back(jaco);

  fs.push_back(distance);
  //  float s = 1;
  //  if (itorCount >= 5) {
  //    s = 1 - 1.8 * fabs(distance);
  //  }
  //  if (s <= 0.1) return false;
  //
  //  vector<double> coeff;
  //  coeff.push_back(coeff_matrix(0, 0));
  //  coeff.push_back(coeff_matrix(0, 1));
  //  coeff.push_back(coeff_matrix(0, 2));
  //  coeff.push_back(s * distance);
  //  coeffs.push_back(coeff);

  pts.push_back(p0);
  return true;
}

//current trans is 0.00115565 0.00115532  -0.142303
//-0.0243705 -0.0981041 -0.0492646
//total dis0.00226888
//real trans is    -0    -0 -0.15
//-0.0149438 -0.0988771          0
int main(int argc, char *argv[]) {
  Eigen::Vector3d p1(0.5, 0.5, 0.5);
  Eigen::Vector3d p2(1.5, 1.5, 1.5);

  Eigen::Matrix3d real_r;
  real_r = Eigen::AngleAxisd(0.15, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d real_t(0, 0.1, 0);

  Sophus::SE3 real_trans(real_r, real_t);

  Eigen::Matrix3d init_r;
  init_r = Eigen::AngleAxisd(0.14, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d init_t(0.01, 0.1, 0.05);
  trans = Sophus::SE3(init_r, init_t);
  trans = trans.inverse();
  // 首先在p1,p2的线上面构造一些点
  uniform_real_distribution<double> u(-5, 5);  //随机数分布对象
  normal_distribution<double> n(0, 0.1);
  default_random_engine e;
  vector<Vector3d> pxs;
  for (int i = 0; i < 30; i++) {
    Eigen::Vector3d px = p1 + (p2 - p1) * u(e);
    Eigen::Vector3d px_(px(0), px(1), px(2));
    px_ = real_trans * px_;
    px(0) = px_(0) ;
    px(1) = px_(1) ;
    px(2) = px_(2) ;
    pxs.push_back(px);
  }

  // 进行30次迭代
  for (int i = 0; i < 30; i++) {
    for (auto pt : pxs) {
      pt = trans * pt;
      AddCornerConstraint(pt, p1, p2, i);
    }
    bool ret = LMOptimization();
    jacobians.clear();
    fs.clear();
    pts.clear();
  }
  std::cout << "real trans is " << real_trans.inverse() << std::endl;
}
