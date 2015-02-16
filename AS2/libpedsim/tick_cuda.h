#ifndef _tick_cuda_h_
#define _tick_cuda_h_

#include <iostream>
__global__ void mAdd(float*,float*,float*,float*);

__global__ void mComputeDirection(float *mDestX, float *mDestY,
				  float *mDestR, float *mAgentsX, 
				  float *mAgentsY, float *mLastDestX,
				  float *mLastDestY, float *mLastDestR,
				  float *mMoveForceX, float *mMoveForceY,
				  float *mCurrWay,
				  float *mWaypointX, float *mWaypointY,
				  float *mWaypointR, float mNumberOfWaypoints);
__device__ void mSetNextDestination(float *mCurrWay, float *mDestX, 
				    float *mDestY, float *mDestR,
				    float *mWaypointX,float *mWaypointY, 
				    float *mWaypointR, float mNumberOfWaypoints);

__device__ void mSetForce(float *mTempX, float *mTempY, float *mTempZ, 
			  float *mX, float *mY,  bool *reached,
			  float *mMoveForceX, float *mMoveForceY);
__device__ float mVectorLength(float mX, float mY, float mZ);
__device__ void mVectorNormalized(float *mX, float *mY,
				  float *mZ, float length);

void kernel_wtg(int blocksPerGrid, int threadsPerBlock,
	       float *d_DestX, float *d_DestY, float *d_DestR, float *d_AgentsX, 
	       float *d_AgentsY, float *d_LastDestX, float *d_LastDestY, 
	       float *d_LastDestR, float *d_MoveForceX, float *d_MoveForceY,
	       float *d_CurrWay, float *d_WaypointX, float *d_WaypointY,
	       float *d_WaypointR, float *d_NumberOfWaypoints);
  

void kernel_go(int blocksPerGrid, int threadsPerBlock,
     		float *d_AgentsX, float *d_MoveForceX,
     		float *d_AgentsY, float *d_MoveForceY);



#endif
