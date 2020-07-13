//
// Created by kilox on 20-6-22.
//
#include <ceres/ceres.h>

// 数值求导, 很多时候我们无法定义一个模板类型的costfunc

// The use of C++ templates makes automatic differentiation efficient, whereas numeric differentiation is expensive, prone to numeric errors, and leads to
// slower convergence.
// 但是数值求导很慢
struct MyCostFunc {
  bool operator()(const double *const x, double *residual) const {
    residual[0] = 10.0 - x[0];
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
