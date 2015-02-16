#include <cstdio>
#include <iostream>
#include "ped_crowd.h"
#include "tick_cuda.h"
#include <cuda_runtime.h>
using namespace std;


__global__ void mAdd(float *A, float *B,float *C, float *D){

	   printf("Hello world from GPU!");     
  int id = blockDim.x * blockIdx.x + threadIdx.x;
  A[id] = llroundf(A[id] + B[id]);
  C[id] = llroundf(C[id] + D[id]);


}

__global__ void mComputeDirection(float *mDestX, float *mDestY,
				  float *mDestR, float *mAgentsX, 
				  float *mAgentsY, float *mLastDestX,
				  float *mLastDestY, float *mLastDestR,
				  float *mMoveForceX, float *mMoveForceY,
				  float *mCurrWay,
				  float *mWaypointX, float *mWaypointY,
				  float *mWaypointR, float mNumberOfWaypoints) {

  int Offset = blockDim.x * blockIdx.x + threadIdx.x;
  float zero = 0;
  bool reachesDest = false; 
  mSetForce(&mDestX[Offset], &mDestY[Offset], &mDestR[Offset], 
	    &mAgentsX[Offset], &mAgentsY[Offset], &reachesDest,
	    &mMoveForceX[Offset], &mMoveForceY[Offset]);

  if (reachesDest == true) {

    mLastDestX[Offset] = mDestX[Offset];
    mLastDestY[Offset] = mDestY[Offset];
    mLastDestR[Offset] = mDestR[Offset];
    mSetNextDestination(&mCurrWay[Offset], &mDestX[Offset], 
		       &mDestY[Offset], &mDestR[Offset],
		       &mWaypointX[Offset], &mWaypointY[Offset], 
		       &mWaypointR[Offset], mNumberOfWaypoints);
  }
 
  int length = mVectorLength(mMoveForceX[Offset], mMoveForceY[Offset], 
			     zero);
  mVectorNormalized(&mMoveForceX[Offset], &mMoveForceY[Offset], 
		    &zero, length);
  
}
__device__ void mSetNextDestination(float *mCurrWay, float *mDestX, 
				    float *mDestY, float *mDestR,
				    float *mWaypointX,float *mWaypointY, 
				    float *mWaypointR, float mNumberOfWaypoints) {
  
  *mCurrWay++;
  int NextWaypoint = fmodf(*mCurrWay, mNumberOfWaypoints);
  *mDestX = mWaypointX[NextWaypoint];
  *mDestY = mWaypointY[NextWaypoint];
  *mDestR = mWaypointR[NextWaypoint];
  

}

__device__ void mSetForce(float *mTempX, float *mTempY, float *mTempZ, 
			  float *mX, float *mY,  bool *reached,
			  float *mMoveForceX, float *mMoveForceY){


  float diffX = *mTempX - *mX;
  float diffY = *mTempX - *mY;;
  float diffZ = 0;
  float length = mVectorLength(diffX, diffY, diffZ);

  //Check which vectors have arrived at dest
  if(length < *mTempZ)
    *reached = true;
  else
    *reached = false;


  mVectorNormalized(&diffX, &diffY, &diffZ, length);
  //Update MoveForce
  *mMoveForceX = diffX;
  *mMoveForceY = diffY;
  
}
__device__ float mVectorLength(float mX, float mY, float mZ){

  return mX*mX + mY*mY + mZ*mZ;
  
}
__device__ void mVectorNormalized(float *mX, float *mY,
				  float *mZ, float length){
  *mX = *mX/length;
  *mY = *mY/length;
  *mZ = *mZ/length;


}
void kernel_go(int blocksPerGrid, int threadsPerBlock,
     		float *d_AgentsX, float *d_MoveForceX,
     		float *d_AgentsY, float *d_MoveForceY){

  mAdd<<<blocksPerGrid, threadsPerBlock>>>(d_AgentsX, d_MoveForceX,
					   d_AgentsY, d_MoveForceY);
cudaError_t err = cudaGetLastError();
if (err != cudaSuccess) 
    printf("Error: %s\n", cudaGetErrorString(err));

}
void kernel_wtg(int blocksPerGrid, int threadsPerBlock,
     		float *d_DestX, float *d_DestY, float *d_DestR, float *d_AgentsX, 
		float *d_AgentsY, float *d_LastDestX, float *d_LastDestY, 
		float *d_LastDestR, float *d_MoveForceX, float *d_MoveForceY,
		float *d_CurrWay, float *d_WaypointX, float *d_WaypointY,
		float *d_WaypointR, float *d_NumberOfWaypoints){

		printf("hello\n");
  mComputeDirection<<<blocksPerGrid, threadsPerBlock>>>(d_DestX, d_DestY,
							d_DestR, d_AgentsX, 
							d_AgentsY, d_LastDestX,
							d_LastDestY, d_LastDestR,
							d_MoveForceX, d_MoveForceY,
							d_CurrWay,
							d_WaypointX, d_WaypointY,
							d_WaypointR, 
							*d_NumberOfWaypoints);
cudaError_t err = cudaGetLastError();
if (err != cudaSuccess) 
    printf("Error: %s\n", cudaGetErrorString(err));

}
