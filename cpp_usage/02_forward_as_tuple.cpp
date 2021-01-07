//
// Created by kilox on 20-11-13.
//

#include <iostream>
#include <tuple>

using namespace std;
int main(int argc, char *argv[]) {
  auto x = make_tuple(1, 2, 3);
  cout << std::get<0>(x) << endl;


  int name=10, age=20, phone=30;
  auto y = std::tie(name, age, phone);

  cout << name << age << phone << endl;
  std::get<0>(y) = 1000;
  cout << name << age << phone << endl;


  int t1,t2,t3;
  std::tie(t1,t2,t3) = y;
  cout << t1 << t2 << t3 << endl;
  //  auto a = std::forward_as_tuple(10, "sdf");
  //  auto b = std::forward_as_tuple(10, "adf");
  //
  //  int num1, num2;
  //  string str1, str2;
  //  std::tie(num1, str1) = a;
  //  std::tie(num2, str2) = b;
  //
  //  cout << "num " << num1 << " " << num2 << endl;
  //  cout << "str " << str1 << " " << str2 << endl;

  //  cout << a << endl;
}