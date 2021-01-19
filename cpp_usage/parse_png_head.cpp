//
// Created by kilox on 21-1-14.
//

#include <stdio.h>
#include <stdlib.h>
#include <cstring>
#include <iostream>

using namespace std;
typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;

struct png_temp_header {
  u16 width;    ///< 宽度
  u16 height;   ///< 高度
  s32 version;  ///< 版本
  u8 reserved[24];
};

typedef struct _DataFrameHeader {
  u16 Width;    ///< 宽度
  u16 Height;   ///< 高度
  u32 ComSize;  ///< 压缩大小
  u8 DataType;  ///< 数据类型
  u8 ComType;   ///< 压缩类型
  u16 Index;    ///< 帧索引

  /**
  全帧温度以16位有符号整数数组表示, 温度浮点值=温度整数值/Slope+Offset
  */
  u16 Slope;
  s16 Offset;

  s32 FPATemp;    ///< 探测器温度, 内部使用
  s32 ShellTemp;  ///< 等效机壳温度, 内部使用

  u8 pad;
  u8 GpioInput0;  ///< GPIO输入0
  u8 GpioInput1;  ///< GPIO输入1
  u8 pad2[5];
  s64 Timestamp;  ///< 时间戳, 单位100ns
  u8 reserved[88];
} DataFrameHeader;

ostream &operator<<(ostream &os, png_temp_header png_temp_header1) {
  os << " width is " << png_temp_header1.width << " height " << png_temp_header1.height << " version " << png_temp_header1.version << endl;
  return os;
}

int main(int argc, char *argv[]) {
  FILE *fp;

  fp = fopen("/home/kilox/ys_latest_img.jpg", "r");

  png_temp_header png_temp_header1;
  int struct_size = sizeof(png_temp_header1);
  fseek(fp, -(struct_size + 4), SEEK_END);
  //  fprintf(fp, "%s %s %s %d", "We", "are", "in", 2014);
  char *buf = new char[struct_size];

  // 读取1个struct_size
  fread(buf, struct_size, 1, fp);

  memcpy(&png_temp_header1, buf, struct_size);

  cout << png_temp_header1 << endl;

  int pix_size = png_temp_header1.height * png_temp_header1.width;

  // 解析头部

  // 解析数据
  int temp_data_start = pix_size * 2 + 4 + 32;
  int temp_head_start = temp_data_start + 128;

  fseek(fp, -temp_head_start, SEEK_END);
  DataFrameHeader dfh;
  fread(&dfh, -temp_head_start, 1, fp);

  fseek(fp, -temp_data_start, SEEK_END);
  s16 *temp_data_buf = new s16[pix_size];
  fread(temp_data_buf, 2, pix_size, fp);

  //  全帧温度以16位有符号整数数组表示, 温度浮点值=温度整数值/Slope+Offset
  for (int row = 0; row < png_temp_header1.height; row++) {
    for (int col = 0; col < png_temp_header1.width; col++) {
      int data_index = row * png_temp_header1.height + col;
      s16 data = temp_data_buf[data_index];
      float temperature = (float)data / dfh.Slope + dfh.Offset;
      cout << "for row " << row << " col " << col << " temper is " << temperature << endl;
    }
  }

  png_temp_header png_temp_header_check;
  fread(buf, struct_size, 1, fp);
  memcpy(&png_temp_header_check, buf, struct_size);

  fclose(fp);

  return (0);
}
