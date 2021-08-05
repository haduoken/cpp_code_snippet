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
  ostringstream oss;
  oss << "w234wtetgererw 2sdg 222 erdfsfer \n";

  oss.flush();
  int data_idx = static_cast<int>(oss.tellp());
  vector<uint8_t> v_data;
  v_data.resize(data_idx);

  string a = oss.str();
  for (int i = 0; i < data_idx; i++) {
    v_data[i] = oss.str().c_str()[i];
  }
  //  v_data.assign(oss.str().begin(), oss.str().end());
  //  std::copy(oss.str().c_str(), oss.str().c_str() + data_idx, v_data);
  //  for (auto it = oss.str().begin(); it != oss.str().end(); it++) {
  //    v_data.emplace_back(*it);
  //  }
  //  std::copy(oss.str().begin(), oss.str().end(), v_data.begin());
  cout << " data is ";
  for (auto data : v_data) {
    cout << " " << (char)data;
  }
  cout << endl;

  return 0;
}
