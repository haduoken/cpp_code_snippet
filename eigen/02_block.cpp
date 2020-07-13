#include <Eigen/Core>
#include <iostream>

using namespace std;
int main() {
  Eigen::Matrix3i A;
  A << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  // 0,0开始, 大小与2,2
  auto B = A.block(0, 0, 2, 2);
  cout << A << endl << B << endl;
  // 大小2,2 , 起始1,1静态大小, 编译期优化
  A.block<2, 2>(1, 1) = Eigen::Matrix2i::Zero();
  cout << A << endl;

  // 取块之后, 操作B也会操作A
  B = Eigen::Matrix2i::Ones();
  cout << A << endl;
}