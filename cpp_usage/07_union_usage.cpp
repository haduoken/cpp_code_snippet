//
// Created by kilox on 21-6-12.
//
#include <stdint.h>
#include <iostream>
using namespace std;

union u_16_data {
 public:
  u_16_data(int d) : data(d) {}
  uint16_t data;
  uint8_t data_arr[2];
  friend std::ostream &operator<<(std::ostream &os, u_16_data &data) {
    os << data.data;
    return os;
  }
};
union i_16_data {
 public:
  i_16_data() : data(0) {}
  i_16_data(int d) : data(d) {}
  int16_t data;
  uint8_t data_arr[2];
  friend std::ostream &operator<<(std::ostream &os, i_16_data &data) {
    os << "0x" << std::hex << data.data;
    return os;
  }
};

int main(int argc, char *argv[]) {
  int a = 0xFF38;
  i_16_data data;

  data.data_arr[0] = 0x38;
  data.data_arr[1] = 0xFF;

  float BB = data.data * 0.01;

  cout << "result is " << data.data << " a is " << a << " BB is " << BB << endl;
}