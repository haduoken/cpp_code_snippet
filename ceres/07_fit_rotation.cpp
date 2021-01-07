#include "ceres_common.h"
#include "eigenpose.h"

#include <iostream>
using namespace std;

void TestRLimit() {
  // 构建problem
  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);
  //  ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
  ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
  // problem.AddParameterBlock(parameters, 4, q_parameterization);

  // 待估计的q
  Eigen::Quaterniond real_q = Eigen::Quaterniond(Eigen::AngleAxisd(1.5, Eigen::Vector3d::UnitZ())) *
                              Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY())) *
                              Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()));

  // 初值为任意
  Eigen::Quaterniond init_q = Eigen::Quaterniond(Eigen::AngleAxisd(-0.9, Eigen::Vector3d::UnitZ())) *
                              Eigen::Quaterniond(Eigen::AngleAxisd(1.6, Eigen::Vector3d::UnitY())) *
                              Eigen::Quaterniond(Eigen::AngleAxisd(1.3, Eigen::Vector3d::UnitX()));

  double para_rot[4] = {0};
  para_rot[0] = init_q.w();
  para_rot[1] = init_q.x();
  para_rot[2] = init_q.y();
  para_rot[3] = init_q.z();

  ceres::LocalParameterization *local_parameterization = new ceres::QuaternionParameterization();
  problem.AddParameterBlock(para_rot, 4, local_parameterization);

  ceres::CostFunction *factor = RLimitErrorV2::Create(real_q.w(), real_q.x(), real_q.y(), real_q.z());
  problem.AddResidualBlock(factor, nullptr, para_rot);

  // 使用
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << " \n";
  //  std::cout << " x : " << initail_x << " -> " << x << "\n";
  cout << " final param is ";

  Eigen::Quaterniond out_q(para_rot[0], para_rot[1], para_rot[2], para_rot[3]);

  Eigen::Quaterniond relative_q = real_q.inverse() * out_q;

  Eigen::Matrix3d R_ = relative_q.toRotationMatrix();
  Eigen::Vector3d RPY;
  RPY(1) = asin(-R_(2, 0));
  double c_pitch = cos(RPY(1));
  RPY(0) = atan2(R_(2, 1) / c_pitch, R_(2, 2) / c_pitch);
  RPY(2) = atan2(R_(1, 0) / c_pitch, R_(0, 0) / c_pitch);

  cout << "final relative RPY is " << RPY << endl;
}

void TestR() {
  // 构建problem
  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);
  //  ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
  ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
  // problem.AddParameterBlock(parameters, 4, q_parameterization);

  // 待估计的q
  Eigen::Quaterniond real_q = Eigen::Quaterniond(Eigen::AngleAxisd(-0.9, Eigen::Vector3d::UnitZ())) *
                              Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY())) *
                              Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()));

  // 初值为任意
  Eigen::Quaterniond init_q = Eigen::Quaterniond(Eigen::AngleAxisd(1.5, Eigen::Vector3d::UnitZ())) *
                              Eigen::Quaterniond(Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitY())) *
                              Eigen::Quaterniond(Eigen::AngleAxisd(1.3, Eigen::Vector3d::UnitX()));

  double para_rot[4] = {0};
  para_rot[0] = init_q.w();
  para_rot[1] = init_q.x();
  para_rot[2] = init_q.y();
  para_rot[3] = init_q.z();

  ceres::LocalParameterization *local_parameterization = new ceres::QuaternionParameterization();
  problem.AddParameterBlock(para_rot, 4, local_parameterization);

  ceres::CostFunction *factor = RError::Create(real_q.w(), real_q.x(), real_q.y(), real_q.z());
  problem.AddResidualBlock(factor, nullptr, para_rot);

  // 使用
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << " \n";
  //  std::cout << " x : " << initail_x << " -> " << x << "\n";
  cout << " final param is ";

  Eigen::Quaterniond out_q(para_rot[0], para_rot[1], para_rot[2], para_rot[3]);

  Eigen::Matrix3d R_ = out_q.toRotationMatrix();
  Eigen::Vector3d RPY;
  RPY(1) = asin(-R_(2, 0));
  double c_pitch = cos(RPY(1));
  RPY(0) = atan2(R_(2, 1) / c_pitch, R_(2, 2) / c_pitch);
  RPY(2) = atan2(R_(1, 0) / c_pitch, R_(0, 0) / c_pitch);

  cout << "final RPY is " << RPY << endl;
}

int main(int argc, char *argv[]) {
  TestRLimit();
}