#pragma once
#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
using namespace std;

class MeasureSpeed {
 public:
  /*
   * 返回true代表观测结束
   * */
  bool operator()(double tms, Eigen::Vector3d &pos, Eigen::Vector3d &out_v) {
    time_e = tms;
    pos_e = pos;

    if (time_e - time_s >= duration) {
      // 计算平均速度
      out_v = (pos_e - pos_s) / (time_e - time_s);
      return true;
    }
    return false;
  }

  void init(double tms, Eigen::Vector3d &pos, double dt) {
    time_s = tms;
    pos_s = pos;
    duration = dt;
  }

  double time_s, time_e;
  double duration = -1;
  Eigen::Vector3d pos_s, pos_e;
};

int main(int argc, char *argv[]) {
  MeasureSpeed ms;

  double duration = 30;
  for (int i = 0; i < 100; i++) {
    Eigen::Vector3d pos(2 * i, 3 * i, 4 * i);
    Eigen::Vector3d out_v;
    if (i % 10 == 0) {
      ms.init(i, pos, 6);
    }
    if (ms(i, pos, out_v)) {
      cout << "v is " << out_v << endl;
    }
  }
  return 0;
}
