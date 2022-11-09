#include <stdio.h>
#include "cuda_disparity_expansion.h"

__global__
void fg_bg_u(int table_rows, int u_cols, int v_cols, int disp_rows, int disp_cols,
	     float padding, float SCALE, int lut_max_disparity, float pixel_error, float robot_radius,
	     float bg_multiplier, float baseline, float fx,
	     float* disparity, cell* table_u, cell* table_v, float* disparity_fg, float* disparity_bg);

__global__
void fg_bg_v(int table_rows, int u_cols, int v_cols, int disp_rows, int disp_cols,
	     float padding, float SCALE, int lut_max_disparity, float pixel_error, float robot_radius,
	     float bg_multiplier, float baseline, float fx,
	     float* disparity, float* disparity_background, cell* table_u, cell* table_v, float* disparity_fg, float* disparity_bg);

CudaDisparityExpansion::CudaDisparityExpansion(){
  disparity_allocated = false;
}

void CudaDisparityExpansion::upload_LUT(cell* _table_u, cell* _table_v, int _table_rows, int _u_cols, int _v_cols){
  table_rows = _table_rows;
  u_cols = _u_cols;
  v_cols = _v_cols;
  
  /*for(int r = 0; r < table_rows; r++){
    for(int c = 0; c < u_cols; c++){
      printf("%d-%d ", _table_u[r*u_cols + c].idx1, _table_u[r*u_cols + c].idx2);
    }
    printf("\n");
  }*/
  
  cudaMalloc(&table_u, _table_rows*_u_cols*sizeof(cell));
  cudaMalloc(&table_v, _table_rows*_v_cols*sizeof(cell));
  cudaMemcpy(table_u, _table_u, _table_rows*_u_cols*sizeof(cell), cudaMemcpyHostToDevice);
  cudaMemcpy(table_v, _table_v, _table_rows*_v_cols*sizeof(cell), cudaMemcpyHostToDevice);
}

void CudaDisparityExpansion::upload_disparity(unsigned char* data, int rows, int cols){
  disp_rows = rows;
  disp_cols = cols;
  
  if(!disparity_allocated){
    cudaMalloc(&disparity, rows*cols*sizeof(float));
    cudaMalloc(&disparity_fg, rows*cols*sizeof(float));
    cudaMalloc(&disparity_bg, rows*cols*sizeof(float));
    disparity_allocated = true;
  }
  
  cudaMemcpy(disparity, data, rows*cols*sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(disparity_fg, data, rows*cols*sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(disparity_bg, data, rows*cols*sizeof(float), cudaMemcpyHostToDevice);
}

void CudaDisparityExpansion::compute_fg_bg(float padding, float SCALE, int lut_max_disparity,
					   float pixel_error, float robot_radius, float bg_multiplier,
					   float baseline, float fx,
					   unsigned char* data, int rows, int cols){
  // upload the disparity image
  disp_rows = rows;
  disp_cols = cols;
  
  if(!disparity_allocated){
    cudaMalloc(&disparity, rows*cols*sizeof(float));
    cudaMalloc(&disparity_background, rows*cols*sizeof(float));
    cudaMalloc(&disparity_fg, rows*cols*sizeof(float));
    cudaMalloc(&disparity_bg, rows*cols*sizeof(float));
    disparity_allocated = true;
  }
  
  cudaMemcpy(disparity, data, rows*cols*sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(disparity_fg, data, rows*cols*sizeof(float), cudaMemcpyHostToDevice);
  cudaMemcpy(disparity_bg, data, rows*cols*sizeof(float), cudaMemcpyHostToDevice);
  
  // fill out the fg and bg disparity images, iterating through columns/x direction
  dim3 threads_per_block(16);
  dim3 num_blocks_u(disp_rows/threads_per_block.x + 1);
  fg_bg_u<<<num_blocks_u, threads_per_block>>>(table_rows, u_cols, v_cols, disp_rows, disp_cols,
					       padding, SCALE, lut_max_disparity, pixel_error, robot_radius,
					       bg_multiplier, baseline, fx, disparity, table_u, table_v,
					       disparity_fg, disparity_bg);
  
  // update the disparity and background images
  cudaMemcpy(disparity, disparity_fg, rows*cols*sizeof(float), cudaMemcpyDeviceToDevice);
  cudaMemcpy(disparity_background, disparity_bg, rows*cols*sizeof(float), cudaMemcpyDeviceToDevice);
  
  // fill out the fg and bg disparity images, iterating through rows/y direction
  dim3 num_blocks_v(disp_cols/threads_per_block.x + 1);
  fg_bg_v<<<num_blocks_v, threads_per_block>>>(table_rows, u_cols, v_cols, disp_rows, disp_cols,
					       padding, SCALE, lut_max_disparity, pixel_error, robot_radius,
					       bg_multiplier, baseline, fx, disparity, disparity_background, table_u, table_v,
					       disparity_fg, disparity_bg);
  
}

void CudaDisparityExpansion::download_fg_bg(unsigned char* fg, unsigned char* bg){
  cudaMemcpy(fg, disparity_fg, disp_rows*disp_cols*sizeof(float), cudaMemcpyDeviceToHost);
  cudaMemcpy(bg, disparity_bg, disp_rows*disp_cols*sizeof(float), cudaMemcpyDeviceToHost);
}

__global__
void fg_bg_u(int table_rows, int u_cols, int v_cols, int disp_rows, int disp_cols,
	     float padding, float SCALE, int lut_max_disparity, float pixel_error, float robot_radius,
	     float bg_multiplier, float baseline, float fx,
	     float* disparity, cell* table_u, cell* table_v, float* disparity_fg, float* disparity_bg){
  int v = blockIdx.x*blockDim.x + threadIdx.x;
  
  // return if the index is out of bounds
  if(v >= disp_rows)
    return;
  
  for(int u = disp_cols - 1; u >= 0; u--){
    float disparity_value = disparity[v*disp_cols + u];
    if(!isnan(double(disparity_value)) &&
       ((int(disparity_value*SCALE)+1) < lut_max_disparity) &&
       ((int(disparity_value*SCALE)+1) > 0)){
      // get bounds
      unsigned int u1 = table_u[u_cols*(int(disparity_value*SCALE)+1) + u].idx1; //maybe add pixel_error
      unsigned int u2 = table_u[u_cols*(int(disparity_value*SCALE)+1) + u].idx2;
      
      // find min and max in bounds
      float u_max = 0.f;
      int max_idx = u-1;//0;
      for(int t = u1; t < u2; t++){
	float value = disparity[v*disp_cols + t];
	if(value > u_max){
	  u_max = value;
	  max_idx = t;
	}
      }
      
      // new values
      float disp_new_fg = u_max;
      float disp_to_depth = baseline * fx/u_max;
      float disp_to_depth_orig = disp_to_depth;
      
      // pad
      if(padding < 0.0){
	float range = bg_multiplier * robot_radius;
	float max_depth = 0.0;
	bool found = true;
	int count = 1;
	while(found){
	  found = false;
	  for(int t = u1; t < u2; t++){
	    float val = baseline*fx / disparity[v*disp_cols + t] - disp_to_depth_orig;
	    if(val-val == 0){ // is finite
	      if(val < count*range && val > max_depth){
		found = true;
		max_depth = val;
	      }
	    }
	  }
	  count++;
	}
	disp_to_depth += max_depth;
      }
      else
	disp_to_depth += padding;
      
      float disp_new_bg = baseline * fx/disp_to_depth;
      
      // set values
      for(int t = u1; t < u2; t++){
	int index = v*disp_cols + t;
	disparity_fg[index] = disp_new_fg;
	disparity_bg[index] = disp_new_bg;
      }
      
      // increment u
//      int u_temp = /*u1 + */max_idx;
//      if(u_temp >= u)
//	u = u1;
//      else
//      u = u_temp + 1;
    }
  }
}


__global__
void fg_bg_v(int table_rows, int u_cols, int v_cols, int disp_rows, int disp_cols,
	     float padding, float SCALE, int lut_max_disparity, float pixel_error, float robot_radius,
	     float bg_multiplier, float baseline, float fx,
	     float* disparity, float* disparity_background, cell* table_u, cell* table_v, float* disparity_fg, float* disparity_bg){
  //int v = blockIdx.x*blockDim.x + threadIdx.x;
  int u = blockIdx.x*blockDim.x + threadIdx.x;
  
  // return if the index is out of bounds
  if(u >= disp_cols)
    return;
  
  for(int v = disp_rows - 1; v >= 0; v--){
    float disparity_value = disparity[v*disp_cols + u] + pixel_error;
    if(!isnan(double(disparity_value)) &&
       ((int(disparity_value*SCALE)+1) < lut_max_disparity) &&
       ((int(disparity_value*SCALE)+1) > 0)){
      // get bounds
      unsigned int v1 = table_v[v_cols*(int(disparity_value*SCALE)+1) + v].idx1;
      unsigned int v2 = table_v[v_cols*(int(disparity_value*SCALE)+1) + v].idx2;
      
      // find min and max in bounds
      float v_max = 0.f;
      int max_idx = v-1;//0;
      for(int t = v1; t < v2; t++){
	float value = disparity[t*disp_cols + u];
	if(value > v_max){
	  v_max = value;
	  max_idx = t;
	}
      }
      
      // new values
      float disp_to_depth = baseline * fx/v_max;
      float disp_new_fg = baseline * fx/(disp_to_depth - robot_radius) + pixel_error;
      v_max = 0.f;
      for(int t = v1; t < v2; t++){
	float value = disparity_background[t*disp_cols + u];
	if(value > v_max)
	  v_max = value;
      }
      disp_to_depth = baseline * fx/v_max;
      float disp_to_depth_orig = disp_to_depth;
      
      // pad
      if(padding < 0.0){
	float range = bg_multiplier * robot_radius;
	float max_depth = 0.0;
	bool found = true;
	int count = 1;
	while(found){
	  found = false;
	  for(int t = v1; t < v2; t++){
	    float val = baseline*fx / disparity[t*disp_cols + u] - disp_to_depth_orig;
	    if(val-val == 0){ // is finite
	      if(val < count*range && val > max_depth){
		found = true;
		max_depth = val;
	      }
	    }
	  }
	  count++;
	}
	disp_to_depth += max_depth;
      }
      else
	disp_to_depth += padding;
      
      float disp_new_bg = baseline * fx/(disp_to_depth + robot_radius) - pixel_error;
      disp_new_bg = disp_new_bg < 0.0 ? 0.0001 : disp_new_bg;
      
      // set values
      for(int t = v1; t < v2; t++){
	int index = t*disp_cols + u;
	disparity_fg[index] = disp_new_fg;//max(disp_new_fg, disparity_fg[index]);
	disparity_bg[index] = disp_new_bg;//max(disp_new_bg, disparity_bg[index]);
      }
      
      // increment v
//      int v_temp = /*v1 + */max_idx;
//      if(v_temp >= v)
//	v = v1;
//      else
//	v = v_temp + 1;
    }
  }
}


