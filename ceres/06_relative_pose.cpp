#include "ceres_common.h"
#include "eigenpose.h"

#include <iostream>
using namespace std;

int main(int argc, char *argv[]) {
  // 构建problem
  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);
  //  ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
  ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
  // problem.AddParameterBlock(parameters, 4, q_parameterization);

  // 待估计的q
  Eigen::Quaterniond real_q = Eigen::Quaterniond(Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())) *
                              Eigen::Quaterniond(Eigen::AngleAxisd(0.3, Eigen::Vector3d::UnitY())) *
                              Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()));
  Eigen::Vector3d real_t(2, 3, 4);

  // 初值为任意
  Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ())) *
                         Eigen::Quaterniond(Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitY())) *
                         Eigen::Quaterniond(Eigen::AngleAxisd(1.3, Eigen::Vector3d::UnitX()));

  double para_Pose[7] = {0};
  para_Pose[6] = q.w();
  para_Pose[5] = q.x();
  para_Pose[4] = q.y();
  para_Pose[3] = q.z();

  ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
  problem.AddParameterBlock(para_Pose, 7, local_parameterization);

  ceres::CostFunction *factor = RTLimitRPError::Create(real_t[0], real_t[1], real_t[2], real_q.w(), real_q.x(), real_q.y(), real_q.z(), 0.1, 0.1);
  problem.AddResidualBlock(factor, nullptr, para_Pose);

  // 使用
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << " \n";
  //  std::cout << " x : " << initail_x << " -> " << x << "\n";
  cout << " final param is ";

  Eigen::Quaterniond out_q(para_Pose[6], para_Pose[3], para_Pose[4], para_Pose[5]);
  Eigen::Vector3d out_t(para_Pose[0], para_Pose[1], para_Pose[2]);

  EigenPose real(real_q, real_t);
  EigenPose out(out_q, out_t);

  // 判断求解出来的q是否和真实的q的z轴是重合的
  EigenPose delta = real.inverse() * out;

  cout << "real is " << real << endl;
  cout << "out is " << out << endl;
  cout << "delta is " << delta << endl;
}