//
// Created by kilox on 20-6-22.
//
#include <ceres/ceres.h>

struct MyCostFunc {
  template <typename T>
  bool operator()(const T *const x, T *residual) const {
    residual[0] = T(10.0) - x[0];
    return true;
  }
};

int main(int argc, char *argv[]) {
  // 构建problem
  ceres::Problem problem;

  double initail_x = 5.0;

  // 将自己的函数包装成cost_function
  ceres::CostFunction *cost_function = new ceres::AutoDiffCostFunction<MyCostFunc, 1, 1>(new MyCostFunc);

  double x1 = initail_x;
  double x2 =
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
