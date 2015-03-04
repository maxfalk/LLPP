#ifndef _heat_cuda_h_
#define _heat_cuda_h_

__global__ void setup_heat_map(int *d_heatmap);

__global__ void update_heat_map(int *d_heatmap);

__global__ void update_heat_map_intes(int *d_heatmap, 
				      float *d_DesiredX,
				      float *d_DesiredY,
				      int Agents);

__global__ void update_heat_map_norm(int *d_heatmap,
				     int *d_scaled_heatmap);


__global__ void update_blurr_map(int *d_scaled_heatmap, 
				 int *d_blurred_heatmap);


void kernel_setup_heat_map(int *d_heatmap);


void kernal_update_heat_map(int *d_heatmap, int *d_scaled_heatmap, 
			    int *d_blurred_heatmap, int *d_DesiredX,
			    int *d_DesiredY,
			    int Agents);





#endif
