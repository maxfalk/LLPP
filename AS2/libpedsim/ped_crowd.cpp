#include "ped_crowd.h"
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <xmmintrin.h>
#include <stdint.h>
#include <assert.h>
#include <math.h>

//Constructor
Ped::Crowd::Crowd(int _NumberOfAgents, int _NumberOfWaypoints, int _mode){
  NumberOfAgents = _NumberOfAgents;
  NumberOfWaypoints = _NumberOfWaypoints;
  mode = _mode;
  if(mode == 1){
    printf("Alloc for vectors\n");
    constructor_vector();
  }else{
    constructor();
  }
  

}

//Destructor
Ped::Crowd::~Crowd(){
  //Free all memory 
  delete[] CurrentWaypoint;
  delete[] AgentsX;
  delete[] AgentsY;
  delete[] AgentsZ;
  
  delete[] MoveForceX;
  delete[] MoveForceY;
  delete[] MoveForceZ;
  
  delete[] DestinationX;
  delete[] DestinationY;
  delete[] DestinationR;
  
  delete[] LastDestinationX;
  delete[] LastDestinationY;
  delete[] LastDestinationR;
  
  delete[] WaypointX;
  delete[] WaypointY;
  delete[] WaypointR;


}
void Ped::Crowd::constructor(){

  //Create all arrays dynamical allocated for number of agents
  CurrentWaypoint = new int[NumberOfAgents];

  AgentsX = new float[NumberOfAgents];
  AgentsY = new float[NumberOfAgents];
  AgentsZ = new float[NumberOfAgents];

  MoveForceX = new float[NumberOfAgents];
  MoveForceY = new float[NumberOfAgents];
  MoveForceZ = new float[NumberOfAgents];
  
  DestinationX = new float[NumberOfAgents];
  DestinationY = new float[NumberOfAgents];
  DestinationR = new float[NumberOfAgents];

  LastDestinationX = new float[NumberOfAgents];
  LastDestinationY = new float[NumberOfAgents];
  LastDestinationR = new float[NumberOfAgents];

  WaypointX = new float[NumberOfWaypoints];
  WaypointY = new float[NumberOfWaypoints];
  WaypointR = new float[NumberOfWaypoints];

}
void Ped::Crowd::constructor_vector(){
  vector_alloc((void **)&CurrentWaypoint, NumberOfAgents);

  vector_alloc((void **)&AgentsX, NumberOfAgents);
  vector_alloc((void **)&AgentsY, NumberOfAgents);
  vector_alloc((void **)&AgentsZ, NumberOfAgents);
   
  vector_alloc((void **)&MoveForceX, NumberOfAgents);
  vector_alloc((void **)&MoveForceY, NumberOfAgents);
  vector_alloc((void **)&MoveForceZ, NumberOfAgents);
   
  vector_alloc((void **)&DestinationX, NumberOfAgents);
  vector_alloc((void **)&DestinationY, NumberOfAgents);
  vector_alloc((void **)&DestinationR, NumberOfAgents);
   
  vector_alloc((void **)&LastDestinationX, NumberOfAgents);
  vector_alloc((void **)&LastDestinationY, NumberOfAgents);
  vector_alloc((void **)&LastDestinationR, NumberOfAgents);

  vector_alloc((void **)&WaypointX, NumberOfWaypoints);
  vector_alloc((void **)&WaypointY, NumberOfWaypoints);
  vector_alloc((void **)&WaypointR, NumberOfWaypoints);

	       

}
void Ped::Crowd::init(){

  for(int i = 0; i < NumberOfAgents; i++){
    CurrentWaypoint[i] = 0;

    DestinationX[i] = WaypointX[CurrentWaypoint[i]];
    DestinationY[i] = WaypointY[CurrentWaypoint[i]];
    DestinationR[i] = WaypointR[CurrentWaypoint[i]];
  }

}

void Ped::Crowd::vector_alloc(void **memptr, int NumberOfFloats){
  int err =  posix_memalign(memptr, __alignof__(__m128), 
			NumberOfFloats*sizeof(float));
  if(err != 0)
    fprintf(stderr, "Error while allocating memory!\n");
  
  //Check that it is aligned
  assert(0 == ((intptr_t)*memptr) % __alignof__(__m128));

}
void Ped::Crowd::go(int &Offset){
  if(mode == 1){
    go_vectorized(Offset);
    //check so that there are 4 agents left
    
    Offset +=3;
  }else{
    AgentsX[Offset] = round(AgentsX[Offset] + MoveForceX[Offset]);
    AgentsY[Offset] = round(AgentsY[Offset] + MoveForceY[Offset]);


  }
  
}
void Ped::Crowd::go_vectorized(int Offset){
  /*
  __m128 *mAgentsX = (__m128 *) &AgentsX[Offset]; 
  __m128 *mAgentsY = (__m128 *) &AgentsY[Offset]; 
  __m128 *mMoveForceX = (__m128 *) &MoveForceX[Offset]; 
  __m128 *mMoveForceY = (__m128 *) &MoveForceY[Offset];
  *mAgentsX = _mm_add_ps(*mAgentsX, *mMoveForceX);
  *mAgentsY = _mm_add_ps(*mAgentsY, *mMoveForceY);
  */
  for(int i = 0; i<4; i++){
    AgentsX[Offset+i] = round(AgentsX[Offset+i] + MoveForceX[Offset+i]);
    AgentsY[Offset+i] = round(AgentsY[Offset+i] + MoveForceY[Offset+i]);

  }


}

void Ped::Crowd::where_to_go(int Offset){
  
  if(mode == 1){
    __m128 *mMoveForceX = (__m128 *) &MoveForceX[Offset];
    __m128 *mMoveForceY = (__m128 *) &MoveForceY[Offset];
    __m128 *mMoveForceZ = (__m128 *) &MoveForceZ[Offset];
    __m128 mLength;

    computeDirection_vectorized(Offset);
    vector_length_vectorized(mMoveForceX,mMoveForceY, mMoveForceZ, &mLength);
    set_vector_normalized_vectorized(mMoveForceX,mMoveForceY, mMoveForceZ,
				     &mLength);
  }else{
    computeDirection(Offset);
    set_vector_normalized(MoveForceX, MoveForceY, MoveForceZ, Offset);
  }
}

void Ped::Crowd::setNextDestination(int Offset) {
  
  CurrentWaypoint[Offset]++;
  int NextWaypoint = CurrentWaypoint[Offset]  % NumberOfWaypoints;
  DestinationX[Offset] = WaypointX[NextWaypoint];
  DestinationY[Offset] = WaypointY[NextWaypoint];
  DestinationR[Offset] = WaypointR[NextWaypoint];

}
void Ped::Crowd::setNextDestination_vectorized(int Offset) {
  
  __m128 *mCuWay = (__m128 *) &CurrentWaypoint[Offset];
  __m128 mAdd1 = _mm_set1_ps(1.0f);
  *mCuWay = _mm_add_ps(*mCuWay, mAdd1);

  float tNextWaypointX[4]  __attribute__((aligned(16)));
  float tNextWaypointY[4]  __attribute__((aligned(16)));
  float tNextWaypointR[4]  __attribute__((aligned(16)));
  for(int i=0; i < 4; i++){
    int NextWay = (CurrentWaypoint[Offset + i]  % NumberOfWaypoints);
    tNextWaypointX[i] = WaypointX[NextWay];
    tNextWaypointY[i] = WaypointY[NextWay];
    tNextWaypointR[i] = WaypointR[NextWay];

  }
  __m128 *mNextWayX = (__m128 *) tNextWaypointX;
  __m128 *mNextWayY = (__m128 *) tNextWaypointY;
  __m128 *mNextWayR = (__m128 *) tNextWaypointR;
  
  _mm_store_ps(&DestinationX[Offset], *mNextWayX); 
  _mm_store_ps(&DestinationY[Offset], *mNextWayY); 
  _mm_store_ps(&DestinationR[Offset], *mNextWayR); 

}

void Ped::Crowd::computeDirection(int Offset) {
  

  bool reachesDestination = false; // if agent reaches destination in n
  float tempDestination[3];
  tempDestination[0] = DestinationX[Offset];
  tempDestination[1] = DestinationY[Offset];
  tempDestination[2] = DestinationR[Offset];
  

  set_force(tempDestination, AgentsX, AgentsY,
	    Offset, &reachesDestination);
  
  
    if (reachesDestination == true) {
      
      LastDestinationX[Offset] = DestinationX[Offset];
      LastDestinationY[Offset] = DestinationY[Offset];
      LastDestinationR[Offset] = DestinationR[Offset];
      
      setNextDestination(Offset);
    }
  
  
}
void Ped::Crowd::computeDirection_vectorized(int Offset) {

 bool reachesDestination[4] = {false,false,false,false}; 
  __m128 *mDestX = (__m128*) &DestinationX[Offset]; 
  __m128 *mDestY = (__m128*) &DestinationY[Offset];
  __m128 * mAgentX = (__m128 *) &AgentsX[Offset];
  __m128 * mAgentY = (__m128 *) &AgentsY[Offset];

  set_force_vectorized(mDestX, mDestY, &DestinationR[Offset], mAgentX, mAgentY, 
		       Offset, reachesDestination);


  for(int i = 0; i < 4; i++){
    if (reachesDestination[i] == true) {
      
      LastDestinationX[Offset+i] = DestinationX[Offset+i];
      LastDestinationY[Offset+i] = DestinationY[Offset+i];
      LastDestinationR[Offset+i] = DestinationR[Offset+i];
      
      setNextDestination(Offset+i);
    }
  }
  
}

void Ped::Crowd::set_force(float *temp, float *X, float *Y,
			   int Offset, bool *reached){

  float diff[3];
  diff[0] = temp[0] - X[Offset];
  diff[1] = temp[1] - Y[Offset];
  diff[2] = 0;
  float len = vector_length(diff[0], diff[1], diff[2]);
  if(len < temp[2])
    *reached = true;
  else
    *reached = false;

  set_vector_normalized(&diff[0], &diff[1], &diff[2], 0, len);
  //Update MoveForce
  MoveForceX[Offset] = diff[0];
  MoveForceY[Offset] = diff[1];
  MoveForceZ[Offset] = diff[2];
   

}

void Ped::Crowd::set_force_vectorized(__m128 *mTempX, __m128 *mTempY, float *TempZ, 
				      __m128 *mX, __m128 *mY, 
				      int Offset, bool *reached){

  float lengths[4] __attribute__((aligned(16)));

  __m128 diffX = _mm_sub_ps(*mTempX, *mX);
  __m128 diffY = _mm_sub_ps(*mTempY, *mY);
  __m128 diffZ = _mm_set1_ps(0);
  vector_length_vectorized(&diffX, &diffY, &diffZ , ((__m128*)&lengths));
  __m128 *mlengths = (__m128 *) lengths;    

  //Check which vectors have arrived at dest
  for(int i = 0; i < 4; i++){
    if(lengths[i] < TempZ[i])
      reached[i] = true;
    else
      reached[i] = false;
  }

  set_vector_normalized_vectorized(&diffX, &diffY, &diffZ, mlengths);
  //Update MoveForce
  _mm_store_ps(&MoveForceX[Offset],diffX);
  _mm_store_ps(&MoveForceY[Offset],diffY);
  _mm_store_ps(&MoveForceZ[Offset],diffZ);
  


}


/*-------------VECTOR-------------------------------------*/
float Ped::Crowd::vector_length(float *X,float *Y,float *Z, int Offset){
  return sqrt(X[Offset]*X[Offset]+
	      Y[Offset]*Y[Offset]+
	      Z[Offset]*Z[Offset]);
  
}
void Ped::Crowd::vector_length_vectorized(__m128 *mX, __m128 *mY, __m128 *mZ, __m128 *lengths){

  *lengths = _mm_sqrt_ps(_mm_add_ps( _mm_add_ps(
						_mm_mul_ps(*mX,*mX),
						_mm_mul_ps(*mY,*mY)),
				     _mm_mul_ps(*mZ,*mZ)));
			  
  
}

float Ped::Crowd::vector_length(float X,float Y,float Z){
  return sqrt(X*X+
	      Y*Y+
	      Z*Z);
  
}
void Ped::Crowd::set_vector_normalized(float *X,float *Y,float *Z, 
				       int Offset){
  float len = vector_length(X,Y,Z,Offset);
  if (len != 0){
    X[Offset] /= len; 
    Y[Offset] /= len; 
    Z[Offset] /= len; 
  }else{
    X[Offset] = 0; 
    Y[Offset] = 0; 
    Z[Offset] = 0; 

  }

}
void Ped::Crowd::set_vector_normalized(float *X,float *Y,float *Z, 
				       int Offset, float len){
  if (len != 0){
    X[Offset] /= len; 
    Y[Offset] /= len; 
    Z[Offset] /= len; 
  }else{
    X[Offset] = 0; 
    Y[Offset] = 0; 
    Z[Offset] = 0; 

  }

}
void Ped::Crowd::set_vector_normalized_vectorized(__m128 *mX, __m128 *mY,
						  __m128 *mZ, __m128 *len){
  //What happens with division by zero?
  *mX = _mm_div_ps(*mX,*len);
  *mY = _mm_div_ps(*mY,*len);
  *mZ = _mm_div_ps(*mZ,*len); 

}
