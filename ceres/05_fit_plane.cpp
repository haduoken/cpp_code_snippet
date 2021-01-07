//
// Created by kilox on 20-6-22.
//
#include <iostream>
#include <random>
#include <vector>
#include "ceres_common.h"
#include "eigenpose.h"

using namespace std;

#define RANDOM(x) rand() % x
// 分析求导, 使用自己的表达式

class LidarPlaneCostFunction : public ceres::SizedCostFunction<1, 6> {
 public:
  LidarPlaneCostFunction(Eigen::Vector3d cp, Eigen::Vector3d plane_unit_norm, double negative_OA_dot_norm)
      : cp_(cp), plane_unit_norm_(plane_unit_norm), negative_OA_dot_norm_(negative_OA_dot_norm) {}
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const override {
    //    Eigen::Vector3d lp = (Eigen::AngleAxisd(parameters[0][2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(parameters[0][1], Eigen::Vector3d::UnitY()) *
    //                          Eigen::AngleAxisd(parameters[0][0], Eigen::Vector3d::UnitX())) *
    //                             cp_ +
    //                         Eigen::Vector3d(parameters[0][3], parameters[0][4], parameters[0][5]);
    //    residuals[0] = plane_unit_norm_.dot(lp) + negative_OA_dot_norm_;
    residuals[0] = computeLoss(parameters[0]);

    double ori_param[6];
    ori_param[0] = parameters[0][0];
    ori_param[1] = parameters[0][1];
    ori_param[2] = parameters[0][2];
    ori_param[3] = parameters[0][3];
    ori_param[4] = parameters[0][4];
    ori_param[5] = parameters[0][5];

    Eigen::Vector3d df_dxyz = plane_unit_norm_;

    float srx = sin(parameters[0][0]);
    float crx = cos(parameters[0][0]);
    float sry = sin(parameters[0][1]);
    float cry = cos(parameters[0][1]);
    float srz = sin(parameters[0][2]);
    float crz = cos(parameters[0][2]);

    float a2 = crx * sry * crz + srx * srz;
    float a3 = crx * srz - srx * sry * crz;
    float a5 = crx * sry * srz - srx * crz;
    float a6 = -srx * sry * srz - crx * crz;
    float a8 = crx * cry;
    float a9 = -srx * cry;

    float b1 = -sry * crz;
    float b2 = 0.0;
    float b3 = cry * crz;
    float b4 = -sry * srz;
    float b5 = 0.0;
    float b6 = sry * srz;
    float b7 = -cry;
    float b8 = 0.0;
    float b9 = -sry;

    float c1 = -srz;
    float c2 = -crz;
    float c3 = 0.0;
    float c4 = crz;
    float c5 = -srz;
    float c6 = 0.0;

    float arx = (a2 * cp_[1] + a3 * cp_[2]) * plane_unit_norm_[0] + (a5 * cp_[1] + a6 * cp_[2]) * plane_unit_norm_[1] +
                (a8 * cp_[1] + a9 * cp_[2]) * plane_unit_norm_[2];

    float ary = (b1 * cp_[0] + b2 * cp_[1] + b3 * cp_[2]) * plane_unit_norm_[0] + (b4 * cp_[0] + b5 * cp_[1] + b6 * cp_[2]) * plane_unit_norm_[1] +
                (b7 * cp_[0] + b8 * cp_[1] + b9 * cp_[2]) * plane_unit_norm_[2];

    float arz = (c1 * cp_[0] + c2 * cp_[1] + c3 * cp_[2]) * plane_unit_norm_[0] + (c4 * cp_[0] + c5 * cp_[1] + c6 * cp_[2]) * plane_unit_norm_[1];

    float atx = plane_unit_norm_[0];

    float aty = plane_unit_norm_[1];

    float atz = plane_unit_norm_[2];

    //    double sr = std::sin(parameters[0][3]);
    //    double cr = std::cos(parameters[0][3]);
    //    double sp = std::sin(parameters[0][4]);
    //    double cp = std::cos(parameters[0][4]);
    //    double sy = std::sin(parameters[0][5]);
    //    double cy = std::cos(parameters[0][5]);
    //
    //    double dx_dr = (cy * sp * cr + sr * sy) * cp_.y() + (sy * cr - cy * sr * sp) * cp_.z();
    //    double dy_dr = (-cy * sr + sy * sp * cr) * cp_.y() + (-sr * sy * sp - cy * cr) * cp_.z();
    //    double dz_dr = cp * cr * cp_.y() - cp * sr * cp_.z();
    //
    //    double dx_dp = -cy * sp * cp_.x() + cy * cp * sr * cp_.y() + cy * cr * cp * cp_.z();
    //    double dy_dp = -sp * sy * cp_.x() + sy * cp * sr * cp_.y() + cr * sr * cp * cp_.z();
    //    double dz_dp = -cp * cp_.x() - sp * sr * cp_.y() - sp * cr * cp_.z();
    //
    //    double dx_dy = -sy * cp * cp_.x() - (sy * sp * sr + cr * cy) * cp_.y() + (cy * sr - sy * cr * sp) * cp_.z();
    //    double dy_dy = cp * cy * cp_.x() + (-sy * cr + cy * sp * sr) * cp_.y() + (cy * cr * sp + sy * sr) * cp_.z();
    //    double dz_dy = 0.;

    if (jacobians && jacobians[0]) {
      jacobians[0][0] = 0;
      jacobians[0][1] = 0;
      jacobians[0][2] = arz;
      jacobians[0][3] = atx;
      jacobians[0][4] = aty;
      jacobians[0][5] = 0;

      double new_param[6];
      new_param[0] = ori_param[0] + 0.0001 * jacobians[0][0];
      new_param[1] = ori_param[1] + 0.0001 * jacobians[0][1];
      new_param[2] = ori_param[2] + 0.0001 * jacobians[0][2];
      new_param[3] = ori_param[3] + 0.0001 * jacobians[0][3];
      new_param[4] = ori_param[4] + 0.0001 * jacobians[0][4];
      new_param[5] = ori_param[5] + 0.0001 * jacobians[0][5];
      double new_residual = computeLoss(new_param);
      cout << "residual from " << residuals[0] << " to " << new_residual << endl;
    }

    return true;
  }
  double computeLoss(const double *param) const {
    Eigen::Vector3d lp = (Eigen::AngleAxisd(param[2], Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(param[1], Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(param[0], Eigen::Vector3d::UnitX())) *
                             cp_ +
                         Eigen::Vector3d(param[3], param[4], param[5]);
    return plane_unit_norm_.dot(lp) + negative_OA_dot_norm_;
  }

 private:
  Eigen::Vector3d cp_;
  Eigen::Vector3d plane_unit_norm_;
  double negative_OA_dot_norm_;
};

int main(int argc, char *argv[]) {
  // 构建problem
  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);
  //  ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
  ceres::LossFunction *loss_function = new ceres::CauchyLoss(1.0);
  // problem.AddParameterBlock(parameters, 4, q_parameterization);
  double para_Pose[7] = {0};
  para_Pose[6] = 1;

  ceres::LocalParameterization *local_parameterization = new PoseLocalParameterization();
  problem.AddParameterBlock(para_Pose, 7, local_parameterization);

  Eigen::Quaterniond q = Eigen::Quaterniond(Eigen::AngleAxisd(0.5, Eigen::Vector3d::UnitZ())) *
                         Eigen::Quaterniond(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitY())) *
                         Eigen::Quaterniond(Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX()));
  Eigen::Matrix3d real_r = q.toRotationMatrix();
  Eigen::Vector3d real_t(1, 2, 0);
  EigenPose real_trans(q, real_t);

  EigenPose inv_real_trans = real_trans.inverse();
  for (int i = 0; i < 100; i++) {
    Eigen::Vector4d plane(RANDOM(50), RANDOM(50), RANDOM(50) + 0.2, RANDOM(50));
    Eigen::Vector3d pt(RANDOM(50), RANDOM(50), 0);
    pt[2] = (-plane[3] - pt[0] * plane[0] - pt[1] * plane[1]) / plane(2);

    Eigen::Vector3d pt_out = inv_real_trans * pt;

    // plan

    problem.AddResidualBlock(new Point2PlaneFactor(pt_out, plane), loss_function, para_Pose);
  }
  //
  //  double params_[6] = {0, 0, 0, 0, 0, 0};
  //
  //  vector<Eigen::Vector3d> pts;
  //  for (int i = 0; i < 5; i++) {
  //    double x = i * 0.5;
  //    double z = sqrt(2) - x;
  //    double y = i * 1;
  //    Eigen::Vector3d pt_on_plan(x, y, z);
  //    Eigen::Vector3d view_pt = real_r.inverse() * pt_on_plan + (-real_r.inverse() * real_t);
  //    pts.push_back(view_pt);
  //  }
  //
  //  Eigen::Vector3d norm(sqrt(2) / 2, 0, sqrt(2) / 2);
  //  double negative_OA_dot_norm = -1;
  //
  //  problem.AddParameterBlock(params_, 6);
  //  for (int i = 0; i < 5; i++) {
  //    problem.AddResidualBlock(new LidarPlaneCostFunction(pts[i], norm, negative_OA_dot_norm), nullptr, params_);
  //  }
  //
  //  // 将自己的函数包装成cost_function, 进行中心点求导
  //
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

  EigenPose out_pose(out_q, out_t);
  cout << "out pose is " << out_pose << endl;
  cout << "real pose is " << real_trans << endl;
  //  for (int i = 0; i < 6; i++) {
  //    cout << " " << params_[i];
  //  }
  //  cout << endl;

  return 0;
}
