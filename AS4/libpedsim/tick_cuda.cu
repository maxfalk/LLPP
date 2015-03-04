#include <cstdio>
#include <iostream>
#include "ped_crowd.h"
#include "tick_cuda.h"
#include <cuda_runtime.h>
using namespace std;


__global__ void mAdd(float *A, float *B,float *C, float *D, int N){

  int id = (blockIdx.x * blockDim.x) + threadIdx.x;
  if(id < N){
    A[id] = llroundf(A[id] + B[id]);
    C[id] = llroundf(C[id] + D[id]);
  }
}

__global__ void mComputeDirection(float *mDestX, float *mDestY,
				  float *mDestR, float *mAgentsX, 
				  float *mAgentsY,
				  float *mMoveForceX, float *mMoveForceY,
				  int *mCurrWay,
				  float *mWaypointX, float *mWaypointY,
				  float *mWaypointR, int *mNumberOfWaypoints,
				  int N) {

  int Offset = (blockIdx.x * blockDim.x) + threadIdx.x;
  if(Offset < N){ 
    bool reached = false;
    mSetForce(&mDestX[Offset], &mDestY[Offset], &mDestR[Offset], 
	      &mAgentsX[Offset], &mAgentsY[Offset],
	      &mMoveForceX[Offset], &mMoveForceY[Offset],
	      &reached);

    
    //Check which vectors have arrived at dest
    if (reached == true) {
      
      mSetNextDestination(&mCurrWay[Offset], &mDestX[Offset], 
			  &mDestY[Offset], &mDestR[Offset],
			  mWaypointX, mWaypointY, 
			  mWaypointR, *mNumberOfWaypoints);
    }
    
    float length = mVectorLength(mMoveForceX[Offset], mMoveForceY[Offset]);	
    mVectorNormalized(&mMoveForceX[Offset], &mMoveForceY[Offset], length);
  }

}
__device__ void mSetNextDestination(int *mCurrWay, float *mDestX, 
				    float *mDestY, float *mDestR,
				    float *mWaypointX,float *mWaypointY, 
				    float *mWaypointR, int mNumberOfWaypoints) {
  
  
  *mCurrWay+=1;
  int NextWaypoint = fmodf(*mCurrWay, mNumberOfWaypoints);
  *mDestX = mWaypointX[NextWaypoint];
  *mDestY = mWaypointY[NextWaypoint];
  *mDestR = mWaypointR[NextWaypoint];
  
  

}

__device__ void mSetForce(float *mTempX, float *mTempY, float *mTempZ,
			  float *mX, float *mY,
			  float *mMoveForceX, float *mMoveForceY,
			  bool *reached){


  float diffX = *mTempX - *mX;
  float diffY = *mTempY - *mY;
  float length = mVectorLength(diffX, diffY);
  if(length < *mTempZ)
    *reached = true;
  else
    *reached = false;

  mVectorNormalized(&diffX, &diffY, length);
  //Update MoveForce
  
  *mMoveForceX = diffX;
  *mMoveForceY = diffY;
 
}
__device__ float mVectorLength(float mX, float mY){

  return sqrt(mX*mX + mY*mY);
  
}
__device__ void mVectorNormalized(float *mX, float *mY, float length){

  if(length != 0){
    *mX = *mX/length;
    *mY = *mY/length;
  }else{
    *mX = 0;
    *mY = 0;
  }

}
void kernel_go(int blocksPerGrid, int threadsPerBlock,
     		float *d_AgentsX, float *d_MoveForceX,
     		float *d_AgentsY, float *d_MoveForceY, int N){

  mAdd<<<blocksPerGrid, threadsPerBlock>>>
    (d_AgentsX, d_MoveForceX, d_AgentsY, d_MoveForceY, N);
  cudaError_t err = cudaGetLastError();	  
  if (err != cudaSuccess) 
    printf("Error: %s\n", cudaGetErrorString(err));

}
void kernel_wtg(int blocksPerGrid, int threadsPerBlock,
     		float *d_DestX, float *d_DestY, float *d_DestR, float *d_AgentsX, 
		float *d_AgentsY, float *d_MoveForceX, float *d_MoveForceY,
		int *d_CurrWay, float *d_WaypointX, float *d_WaypointY,
		float *d_WaypointR, int *d_NumberOfWaypoints, int N){

  
  mComputeDirection<<<blocksPerGrid, threadsPerBlock>>>(d_DestX, d_DestY,
							d_DestR, d_AgentsX,	
							d_AgentsY,
							d_MoveForceX, d_MoveForceY,
							d_CurrWay,
							d_WaypointX, d_WaypointY,
							d_WaypointR, 
							d_NumberOfWaypoints, N);
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess) 
    printf("Error: %s\n", cudaGetErrorString(err));


}
