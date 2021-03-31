//
// Created by kilox on 20-11-13.
//

#include <deque>
#include <iostream>
#include <map>
#include <tuple>

using namespace std;
int main(int argc, char *argv[]) {
  map<int, int> mp_foo;

  mp_foo[1] = 10;
  mp_foo[2] = 20;

  cout << mp_foo[1] << endl;

  cout << mp_foo.find(1)->second << endl;

  if (mp_foo.find(2) != mp_foo.end()) {
    cout << "find " << mp_foo.find(2)->second << endl;
  } else {
    cout << "not found" << endl;
  }
  auto a = mp_foo.count(3);

  if (mp_foo.count(3) >= 0) {
    cout << "find " << mp_foo.find(2)->second << endl;
  } else {
    cout << "not found" << endl;
  }
}