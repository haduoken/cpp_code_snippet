//
// Created by kilox on 20-12-9.
//
#include "pcl_common.h"

int main(int argc, char** argv) {
  deque<PclScanPtr> q_scans;

  PointType tmp;
  tmp.x = 10;
  for (int i = 0; i < 100; i++) {
    cout << "for loop " << i << " **************** " << endl;

    PclScanPtr pcl_s(new PclScan());
    for (int j = 0; j < i; j++) {
      pcl_s->points.push_back(tmp);
    }

    if (q_scans.size() >= 10) {
      q_scans.pop_front();
    }
    q_scans.push_back(pcl_s);

    // visit
    for (int k = 0; k < q_scans.size(); k++) {
      cout << " " << q_scans[k]->points.size() << " ";
    }
    cout << endl;
  }
}