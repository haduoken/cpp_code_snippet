//
// Created by kilox on 20-6-22.
//
#include <ceres/ceres.h>

// 分析求导, 使用自己的表达式

class MyCostFunc : public ceres::SizedCostFunction<1, 1> {
 public:
  virtual ~MyCostFunc() {}
  virtual bool Evaluate(double const *const *parameters, double *residuals, double **jacobians) const {
    const double x = parameters[0][0];
    residuals[0] = 10 - x;
    if (jacobians != nullptr && jacobians[0] != nullptr) {
      jacobians[0][0] = -1;
    }
    return true;
  }
};

int main(int argc, char *argv[]) {
  // 构建problem
  ceres::Problem problem;

  double initail_x = 5.0;
  double x = initail_x;

  // 将自己的函数包装成cost_function, 进行中心点求导
  ceres::CostFunction *cost_function = new ceres::NumericDiffCostFunction<MyCostFunc, ceres::CENTRAL, 1, 1>(new MyCostFunc);

  problem.AddResidualBlock(cost_function, nullptr, &x);

  // 使用
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);

  std::cout << summary.BriefReport() << " \n";
  std::cout << " x : " << initail_x << " -> " << x << "\n";
  return 0;
}
