//
// Created by kilox on 20-11-13.
//

#include <deque>
#include <iostream>
#include <tuple>

using namespace std;
int main(int argc, char *argv[]) {
  deque<int> q_nums;

  for (int i = 0; i < 100; i++) {
    if (q_nums.size() > 10) {
      q_nums.pop_front();
    }
    q_nums.push_back(i);

    cout << "for loop " << i << " **************** " << endl;

    for (int j = 0; j < q_nums.size(); j++) {
      cout << " " << q_nums[j] << " ";
    }
    cout << endl;
  }
}