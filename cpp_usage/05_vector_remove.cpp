//

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

int main(int argc, char *argv[]) {
  vector<int> v_1{1, 2, 3};
  vector<int> v_2{1, 2, 3};
  if (v_1 == v_2) {
    cout << "hello world" << endl;
  }
}