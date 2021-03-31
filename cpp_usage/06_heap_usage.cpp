//

#include <algorithm>
#include <deque>
#include <iostream>
#include <map>
#include <tuple>
#include <vector>

using namespace std;
ostream &operator<<(ostream &os, vector<int> v) {
  for (auto &data : v) {
    os << " " << data;
  }
  return os;
}

struct greater1 {
  bool operator()(const double &a, const double &b) const { return a > b; }
};
struct smaller1 {
  bool operator()(const double &a, const double &b) const { return a < b; }
};

int main(int argc, char *argv[]) {
  vector<double> queue;
  queue.push_back(5);

  // greater对应为小根堆, 每放进去一个元素就将
  queue.push_back(1);
  std::push_heap(queue.begin(), queue.end(), greater1());

  queue.push_back(2);
  std::push_heap(queue.begin(), queue.end(), greater1());

  queue.push_back(3);
  std::push_heap(queue.begin(), queue.end(), greater1());

  //  std::pop_heap(queue.begin(), queue.end(), greater1());
  //  queue.pop_back();
  cout << " queue data is ";
  for (auto i : queue) {
    cout << " " << i;
  }
  cout << endl;

  double top = queue[0];
  // pop_heap将堆顶元素放到末尾
  std::pop_heap(queue.begin(), queue.end(), greater1());
  cout << " queue data is ";
  for (auto i : queue) {
    cout << " " << i;
  }
  cout << endl;

  // 使用pop_back将元素抹掉
  queue.pop_back();
}