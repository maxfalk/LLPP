#ifndef _tick_cuda_h_
#define _tick_cuda_h_

#include <iostream>
__global__ void mAdd(float*,float*,float*,float*);

__global__ void mComputeDirection(float *mDestX, float *mDestY,
				  float *mDestR, float *mAgentsX, 
				  float *mAgentsY,
				  float *mMoveForceX, float *mMoveForceY,
				  int *mCurrWay,
				  float *mWaypointX, float *mWaypointY,
				  float *mWaypointR, int mNumberOfWaypoints);
__device__ void mSetNextDestination(int *mCurrWay, float *mDestX, 
				    float *mDestY, float *mDestR,
				    float *mWaypointX,float *mWaypointY, 
				    float *mWaypointR, int mNumberOfWaypoints);

__device__ void mSetForce(float *mTempX, float *mTempY, float *mTempZ,
			  float *mX, float *mY,  
			  float *mMoveForceX, float *mMoveForceY,
			  bool *reached);
__device__ float mVectorLength(float mX, float mY);
__device__ void mVectorNormalized(float *mX, float *mY, float length);

void kernel_wtg(int blocksPerGrid, int threadsPerBlock,
	       float *d_DestX, float *d_DestY, float *d_DestR, float *d_AgentsX, 
	       float *d_AgentsY, float *d_MoveForceX, float *d_MoveForceY,
	       int *d_CurrWay, float *d_WaypointX, float *d_WaypointY,
	       float *d_WaypointR, int *d_NumberOfWaypoints);
  

void kernel_go(int blocksPerGrid, int threadsPerBlock,
     		float *d_AgentsX, float *d_MoveForceX,
     		float *d_AgentsY, float *d_MoveForceY);



#endif
