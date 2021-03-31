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
  vector<int> v_test;
  for (int i = 0; i < 100; i++) {
    v_test.push_back(i);
  }
  cout << v_test << endl;

  int remove_from = 10, remove_to = 20;
  //  for (int i = remove_from; i < remove_to; i++) {
  v_test.erase(v_test.begin() + remove_from, v_test.begin() + remove_to);
  //  }
  cout << v_test << endl;
}