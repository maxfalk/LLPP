#include <cstdio>
#include <iostream>
#include "ped_model.h"
#include "heat_cuda.h"
#include <cuda_runtime.h>
#define WEIGHTSUM 273
using namespace std;
#define debug 1



__global__ void setup_heat_map(int *d_heatmap){

  int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
  int idy = (blockIdx.y * blockDim.y) + threadIdx.y;

  if(idx < SIZE and idy < SIZE){
    d_heatmap[(idx*SIZE)+idy] = 0;
  }

}
__global__ void update_heat_map(int *d_heatmap){

  //Fade out heat
  int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
  int idy = (blockIdx.y * blockDim.y) + threadIdx.y;
  if(idx < SIZE and idy < SIZE){
    d_heatmap[(idy*SIZE)+idx] *= 0.80;

  }
}
__global__ void update_heat_map_intes(int *d_heatmap, 
				      int *d_DesiredX,
				      int *d_DesiredY,
				      int Agents){

  int idx = (blockIdx.x * blockDim.x) + threadIdx.x;

  if(idx < Agents){
    atomicAdd(&d_heatmap[(d_DesiredY[idx]*SIZE)+d_DesiredX[idx]],40);
  }
}
__global__ void update_heat_map_norm(int *d_heatmap,
				     int *d_scaled_heatmap){

  int idx = (blockIdx.x * blockDim.x) + threadIdx.x;
  int idy = (blockIdx.y * blockDim.y) + threadIdx.y;

  if(idx < SIZE and idy < SIZE){
    d_heatmap[(idy*SIZE)+idx] = d_heatmap[(idy*SIZE)+idx] < 255 
				? d_heatmap[(idy*SIZE)+idx]:255;
  
    for(int x = 0; x < CELLSIZE; x++){
      for(int y = 0; y < CELLSIZE; y++){
	int value = d_heatmap[(idy*SIZE)+idx];
	d_scaled_heatmap[((idy*CELLSIZE+y)*SCALED_SIZE)+(idx*CELLSIZE+x)] 
	  = value;
      }
    }
  }
}
__global__ void update_blurr_map(int *d_scaled_heatmap, 
				 int *d_blurred_heatmap){

    const int w[5][5] = {
      {1,4,7,4,1},
      {4,16,26,16,4},
      {7,26,41,26,7},
      {4,16,26,16,4},
      {1,4,7,4,1}
    };

  int idx = (blockIdx.x * blockDim.x) + threadIdx.x + 2;
  int idy = (blockIdx.y * blockDim.y) + threadIdx.y + 2;

  if(idx < SCALED_SIZE-2 and idy < SCALED_SIZE-2){
    int sum = 0;
    for(int k =- 2; k < 3; k++)
      {
	for(int l= -2; l < 3; l++)
	  {
	    sum += w[2 + k][2 + l] * d_scaled_heatmap[(idy+k)*SCALED_SIZE+(idx+l)];
	  }
      }
    
    int value = sum/WEIGHTSUM;
    d_blurred_heatmap[(idy*SCALED_SIZE)+idx] = 0x00FF0000 | value<<24 ;
  
  }

}

void kernel_setup_heat_map(int *d_heatmap){

  dim3 blockHeat(16,16);
  dim3 gridHeat((SIZE+15)/16,(SIZE+15)/16);
  setup_heat_map<<<gridHeat, blockHeat>>>(d_heatmap);
  
}
void kernal_update_heat_map(int *d_heatmap, int *d_scaled_heatmap, 
			    int *d_blurred_heatmap, int *d_DesiredX,
			    int *d_DesiredY,
			    int Agents){

  //Update heat map
  dim3 blockHeat(16,16);
  dim3 gridHeat((SIZE+15)/16,(SIZE+15)/16);
  update_heat_map<<<gridHeat, blockHeat>>>(d_heatmap);
 
  //-----------------------------------------------------
  int threads = 512;
  while(Agents % threads != 0)
    threads--;
  int blocks = Agents / threads;
  update_heat_map_intes<<<blocks, threads>>>(d_heatmap, 
					     d_DesiredX,
					     d_DesiredY,
					     Agents);
  //-----------------------------------------------------
  dim3 blockHeatNorm(16,16);
  dim3 gridHeatNorm((SIZE+15)/16,(SIZE+15)/16);
  update_heat_map_norm<<<gridHeatNorm, blockHeatNorm>>>(d_heatmap,
							d_scaled_heatmap);

  //------------------------------------------------------------------------
  //update blurr
  dim3 blockBlurr(16,16);
  dim3 gridBlurr((SCALED_SIZE+15)/16,(SCALED_SIZE+15)/16);
  update_blurr_map<<<gridBlurr, blockBlurr>>>(d_scaled_heatmap, 
					      d_blurred_heatmap);

}

