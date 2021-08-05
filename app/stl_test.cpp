#include <iostream>

using namespace std;

// 1. pair: 作为一个模板类,使用方法和其他没有区别,
// 2. 拷贝构造函数: 当返回对象的时候,由于编译器优化(RVO: return value opti) 不会调用拷贝构造函数
pair<bool, int> foo() {
  auto a = pair<bool, int>(false, 1);
  auto b = a.first + a.second;
  return {false, 1};
}

// 1. 拷贝构造: 写法
// 2. 赋值构造: 写法
class Foo {
 public:
  Foo(int a, int b) : a_(a), b_(b) { cout << "construct a b" << endl; }

  Foo(const Foo &other) { cout << "copy assignment " << endl; };

  Foo &operator=(const Foo &p) {
    cout << "= assignment " << endl;
    return *this;
  }
  int a_, b_;
};

int main(int argc, char *argv[]) { foo(); }
