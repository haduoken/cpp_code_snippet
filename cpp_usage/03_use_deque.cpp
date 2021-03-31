//
// Created by kilox on 20-11-13.
//

#include <deque>
#include <iostream>
#include <list>
#include <tuple>

using namespace std;
int main(int argc, char *argv[]) {
  //  deque<int> q_nums;
  //
  //  for (int i = 0; i < 100; i++) {
  //    if (q_nums.size() > 10) {
  //      q_nums.pop_front();
  //    }
  //    q_nums.push_back(i);
  //
  //    cout << "for loop " << i << " **************** " << endl;
  //
  //    for (int j = 0; j < q_nums.size(); j++) {
  //      cout << " " << q_nums[j] << " ";
  //    }
  //    cout << endl;
  //  }
  list<int> l_data;
  l_data.push_back(1);
  l_data.push_back(2);
  l_data.push_back(3);
  l_data.push_back(4);
  cout << " front is " << l_data.front() << " back is " << l_data.back() << endl;
  auto iter = l_data.begin();
  l_data.erase(++iter, l_data.end());
  cout << " remain data";
  for (auto i : l_data) {
    cout << " " << i;
  }
  cout << endl;
}