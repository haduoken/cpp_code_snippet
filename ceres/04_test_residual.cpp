//
// Created by kilox on 20-6-22.
//
#include <ceres/ceres.h>
#include <iostream>
using namespace std;

// 分析求导, 使用自己的表达式

class MyCostFunc : public ceres::SizedCostFunction<1, 2> {
 public:
  MyCostFunc(double x, double y) : x_(x), y_(y) {}
  virtual ~MyCostFunc() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    const double k = parameters[0][0];
    const double b = parameters[0][1];

    residuals[0] = k * x_ + b - y_;
    if (jacobians != nullptr && jacobians[0] != nullptr) {
      jacobians[0][0] = x_;
      jacobians[0][1] = 1;

      double new_k = k + jacobians[0][0] * 0.0001;
      double new_b = b + jacobians[0][1] * 0.0001;
      cout << " residual from " << residuals[0] << " to " << new_k * x_ + new_b - y_ << endl;
    }
    return true;
  }

 private:
  double x_, y_;
};

int main(int argc, char *argv[]) {
  // 构建problem
  ceres::Problem::Options problem_options;
  ceres::Problem problem(problem_options);
  ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
  // problem.AddParameterBlock(parameters, 4, q_parameterization);
  double params_[2] = {0, 0};
  problem.AddParameterBlock(params_, 2);

  // 将自己的函数包装成cost_function, 进行中心点求导
  problem.AddResidualBlock(new MyCostFunc(0, -9), loss_function, params_);
  problem.AddResidualBlock(new MyCostFunc(1, -12), loss_function, params_);
  problem.AddResidualBlock(new MyCostFunc(-1, -6), loss_function, params_);
  problem.AddResidualBlock(new MyCostFunc(-4, 3), loss_function, params_);

  // 使用
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.FullReport() << " \n";
  //  std::cout << " x : " << initail_x << " -> " << x << "\n";
  cout << " final param is " << params_[0] << " and " << params_[1] << endl;

  return 0;
}
