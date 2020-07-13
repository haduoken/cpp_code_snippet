#include <Eigen/Core>
#include <iostream>

using namespace std;

int main(int argc, char *argv[]) {
  //构造矩阵, 不赋值会进行随机初始化
  Eigen::Matrix3d A;
  std::cout << A << endl;

  // 使用特殊附近进行赋值, 按行进行赋值
  A << 1.0, 2.0, 3.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
  cout << A << endl;

  // 构造列向量
  Eigen::Vector3d B(1.0, 2.0, 3.0);
  cout << B << endl;

  // 取某个单元的值以及赋值
  B(1) = 10.0;
  cout << B << endl;
  // 先行后列
  A(1, 2) = 10.0;
  cout << A << endl;

  // 特殊矩阵的构建
  auto C = Eigen::Matrix3f::Random();
  auto D = Eigen::Matrix3f::Identity();
  auto E = Eigen::Matrix3f::Zero();
  auto F = Eigen::Matrix3f::Ones();
  auto G = Eigen::Matrix3f::Constant(4.5);
  cout << C << endl << D << endl << E << endl << F << endl << G << endl;
}
