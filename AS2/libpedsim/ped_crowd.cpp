#include "ped_crowd.h"
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <xmmintrin.h>
#include <stdint.h>
#include <assert.h>
#include <math.h>

//Constructor
Ped::Crowd::Crowd(int _NumberOfAgents, int _NumberOfWaypoints){
  NumberOfAgents = _NumberOfAgents;
  NumberOfWaypoints = _NumberOfWaypoints;

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

  for(int i = 0; i < NumberOfAgents; i++){
    CurrentWaypoint[i] = 0;

    AgentsX[i] = 0;
    AgentsY[i] = 0;
    AgentsZ[i] = 0;
    
    MoveForceX[i] = 0;
    MoveForceY[i] = 0;
    MoveForceZ[i] = 0;
    
    DestinationX[i] = 0;
    DestinationY[i] = 0;
    DestinationR[i] = 0;
  
    LastDestinationX[i] = 0;
    LastDestinationY[i] = 0;
    LastDestinationR[i] = 0;


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
void Ped::Crowd::init(){

  for(int i = 0; i < NumberOfAgents; i++){
    DestinationX[i] = WaypointX[CurrentWaypoint[i]];
    DestinationY[i] = WaypointY[CurrentWaypoint[i]];
    DestinationR[i] = WaypointR[CurrentWaypoint[i]];
  }

}

void Ped::Crowd::vector_alloc(void **memptr){
  int err =  posix_memalign(memptr, __alignof__(__m128), 
			NumberOfAgents*sizeof(float));
  if(err != 0)
    fprintf(stderr, "Error while allocating memory!\n");
  
  //Check that it is aligned
  assert(0 == ((intptr_t)*memptr) % __alignof__(__m128));

}
void Ped::Crowd::go(int Offset){

  AgentsX[Offset] = round(AgentsX[Offset] + MoveForceX[Offset]);
  AgentsY[Offset] = round(AgentsY[Offset] + MoveForceY[Offset]);

}


void Ped::Crowd::where_to_go(int Offset){
  
  computeDirection(Offset);
  set_vector_normalized(MoveForceX, MoveForceY, MoveForceZ, Offset);
    
}

void Ped::Crowd::setNextDestination(int Offset) {
  
  CurrentWaypoint[Offset]++;
  int NextWaypoint = CurrentWaypoint[Offset]  % NumberOfWaypoints;
  DestinationX[Offset] = WaypointX[NextWaypoint];
  DestinationY[Offset] = WaypointY[NextWaypoint];
  DestinationR[Offset] = WaypointR[NextWaypoint];

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

/*-------------VECTOR-------------------------------------*/
float Ped::Crowd::vector_length(float *X,float *Y,float *Z, int Offset){
  return sqrt(X[Offset]*X[Offset]+
	      Y[Offset]*Y[Offset]+
	      Z[Offset]*Z[Offset]);
  
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
