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
//Destructor
Ped::Crowd::~Crowd(){
  //Free all memory 

  delete[]  AgentsX;
  delete[]  AgentsY;
  delete[]  AgentsZ;
  
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

void Ped::Crowd::vector_alloc(void **memptr){
  int err =  posix_memalign(memptr, __alignof__(__m128), 
			NumberOfAgents*sizeof(float));
  if(err != 0)
    fprintf(stderr, "Error while allocating memory!\n");
  
  //Check that it is aligned
  assert(0 == ((intptr_t)*memptr) % __alignof__(__m128));

}
void Ped::Crowd::go(int Offset){

  AgentsX[Offset] += MoveForceX[Offset];
  AgentsY[Offset] += MoveForceX[Offset];

}


void Ped::Crowd::where_to_go(int Offset){
    setNextDestination(Offset);
    computeDirection(Offset);
    set_vector_normalized(MoveForceX, MoveForceY, MoveForceZ, Offset);
    
}

void Ped::Crowd::setNextDestination(int Offset) {
  

  int NextWaypoint = (CurrentWaypoint + 1) % NumberOfWaypoints;
  DestinationX[Offset] =  WaypointX[NextWaypoint];
  DestinationY[Offset] = WaypointY[NextWaypoint];
  DestinationR[Offset] = WaypointR[NextWaypoint];

}

void Ped::Crowd::computeDirection(int Offset) {
  
  if (DestinationX == NULL) {
    MoveForceX[Offset] = 0;
    MoveForceY[Offset] = 0;
    MoveForceZ[Offset] = 0;
   
  }else{

    
    bool reachesDestination = false; // if agent reaches destination in n
    if (LastDestinationX == NULL) {
      float tempDestination[3];
      tempDestination[0] = DestinationX[Offset];
      tempDestination[1] = DestinationY[Offset];
      tempDestination[2] = DestinationR[Offset];

      //Set type, do we need type?

      set_force(tempDestination, AgentsX, AgentsY, 
	       Offset, &reachesDestination);
     
    }
    else {
      float tempDestination[3];
      tempDestination[0] = DestinationX[Offset];
      tempDestination[1] = DestinationY[Offset];
      tempDestination[2] = DestinationR[Offset];
      

      set_force(tempDestination, LastDestinationX, LastDestinationY,
	       Offset, &reachesDestination);
    }
    
    if (reachesDestination == true) {
      setNextDestination(Offset);
    }
  }
  
}
void Ped::Crowd::set_force(float *temp, float *X, float *Y,
			   int Offset, bool *reached){

  float diff[3];
  diff[0] = temp[0] - X[Offset];
  diff[1] = temp[1] - Y[Offset];
  diff[2] = 0;
  if(vector_length(diff[0], diff[1], diff[2]) < temp[2])
    *reached = true;
  else
    *reached = false;

  set_vector_normalized(diff[0], diff[1], diff[2]);
  //Update MoveForce
  MoveForceX[Offset] = diff[0];
  MoveForceY[Offset] = diff[1];
  MoveForceZ[Offset] = diff[2];
   

}

/*-------------VECTOR-------------------------------------*/
float Ped::Crowd::vector_dotproduct(int Agent1, int Agent2){
  return (AgentsX[Agent1]*AgentsX[Agent2] +
	  AgentsY[Agent1]*AgentsY[Agent2] +
	  AgentsZ[Agent1]*AgentsZ[Agent2]);
}
float *Ped::Crowd::vector_subtract(int Agent1, int Agent2){
  float *relativeEnd = new float[3];
  relativeEnd[0] = AgentsX[Agent2] - AgentsX[Agent1];
  relativeEnd[1] = AgentsY[Agent2] - AgentsY[Agent2];
  relativeEnd[2] = AgentsZ[Agent2] - AgentsZ[Agent2];
  return relativeEnd;
}

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
void Ped::Crowd::set_vector_normalized(float *X,float *Y,float *Z, int Offset){
  float len = vector_length(X, Y, Z, Offset);
  if (len != 0){
    X[Offset] /= len; 
    Y[Offset] /= len; 
    Z[Offset] /= len; 
  }

}
void Ped::Crowd::set_vector_normalized(float X,float Y,float Z){
  float len = vector_length(X, Y, Z);
  if (len != 0){
    X /= len; 
    Y /= len; 
    Z /= len; 
  }

}
