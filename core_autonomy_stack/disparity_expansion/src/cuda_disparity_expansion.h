/**
 * @attention Copyright (C) 2017
 * @attention Carnegie Mellon University
 * @attention All rights reserved
 *
 * @author: AirLab / Field Robotics Center
 * @author: John Keller
 *
 * @attention This code was developed under award #A018532.
 *
 */
#ifndef CUDA_DISPARITY_EXPANSION_H_
#define CUDA_DISPARITY_EXPANSION_H_

#include <stdio.h>

struct cell {
  unsigned int idx1;
  unsigned int idx2;
};

class CudaDisparityExpansion{
 private:
  // LUT
  int table_rows, u_cols, v_cols;
  cell* table_u;
  cell* table_v;
  
  // parameters
  float padding, SCALE;
  
  // disparity
  bool disparity_allocated;
  float* disparity;
  float* disparity_background;
  float* disparity_fg;
  float* disparity_bg;
  int disp_rows, disp_cols;
  
 public:
  CudaDisparityExpansion();
  void upload_LUT(cell*, cell*, int, int, int);
  void upload_disparity(unsigned char*, int, int);
  void compute_fg_bg(float, float, int, float, float, float, float, float, unsigned char*, int, int);
  void download_fg_bg(unsigned char*, unsigned char*);
};

class CudaZeroCopyMemory{
public:
  unsigned char* data;
  CudaZeroCopyMemory(int);
};

#endif
