#include "ped_crowd.h"
#include "ped_waypoint.h"
#include <stdlib.h>
#include <iostream>

//Constructor
Ped::Crowd::Crowd(){
  //Create all arrays dynamical allocated for number of agents

}
//Destructor
Ped::Crowd::~Crowd(){
  //Free all memory 

}

void Ped::Crowd::where_to_go(int Offset){
    setNextDestination(Offset);
    computeDirection(Offset);
    set_vector_normalized(MoveForceX, MoveForceY, MoveForceZ, Offset)
    
    

}

void Ped::Crowd::setNextDestination(int Offset) {
  

  int NextWaypoint = (CurrentWaypoint + 1) % NumberOfWaypoints;
  _DestinationX[DestinationOffset] =  WaypointX[NextWaypoint];
  _DestinationY[DestinationOffset] = WaypointY[NextWaypoint];
  _DestinationR[DestinationOffset] = WaypointR[NextWaypoint];

}

void Ped::Crowd::computeDirection(int Offset) {
  
  if (*DestinationX == NULL) {
    MoveForceX[Offset] = 0;
    MoveForceY[Offset] = 0;
    MoveForceZ[Offset] = 0;
   
  }else{

    
    bool reachesDestination = false; // if agent reaches destination in n
    if (*LastDestinationX[Offset] == NULL) {
      float tempDestination[3];
      tempDestination[0] = DestinationX[Offset];
      tempDestination[1] = DestinationY[Offset];
      tempDestination[2] = DestinationR[Offset];

      //Set type, do we need type?

      setForce(tempDestination, AgentsX, AgentsY, 
	       Offset, &reachesDestination);
     
    }
    else {
      float tempDestination[3];
      tempDestination[0] = DestinationX[Offset];
      tempDestination[1] = DestinationY[Offset];
      tempDestination[2] = DestinationR[Offset];
      

      getForce(tempDestination, LastDestinationX, LastDestinationY,
	       Offset, &reachesDestination);
    }
    
    if (reachesDestination == true) {
      setNextDestination(Offset)
    }
  }
  
}
void Ped::Crowd::set_force(float *temp, float *X, float *Y,
			   int Offset, bool *reached){

  float diff[3];
  diff[0] = temp[0] - X[Offset];
  diff[1] = temp[1] - Y[Offset];
  diff[2] = 0;
  if(vector_length(diff) < temp[2])
    *reached = true;
  else
    *reached = false;

  set_vector_normalized(diff[0], diff[1], diff[2], 0);
  //Update MoveForce
  MoveForceX[Offset] = diff[0];
  MoveForceY[Offset] = diff[1];
  MoveForceZ[Offset] = diff[2];
   

}
/*--------------------WAYPOINT------------------------------------*/
void Ped::Crowd::waypoint_normalpoint(int Agent1, int Agent2){
  float* relativeEnd = vector_subtract(Agent2,Agent1);
  //TBI
  
}
/*-------------VECTOR-------------------------------------*/
float Ped::Crowd::vector_dotproduct(int Agent1, int Agent2){
  return (AgentsX[Agent1]*AgentsX[Agent2] +
	  AgentsY[Agent1]*AgentsY[Agent2] +
	  AgentsZ[Agent1]*AgentsZ[Agent2]);
}
float *Ped::Crowd::vector_subtract(int Agent1, int Agent2){
  float relativeEnd = new float[3];
  relativeEnd[0] = AgentsX[Agent2] - AgentsX[Agent1];
  relativeEnd[1] = AgentsY[Agent2] - AgentsY[Agent2];
  relativeEnd[2] = AgentsZ[Agent2] - AgentsZ[Agent2];
  return relativeEnd;
}

float *Ped::Crowd::vector_length(float *X,float *Y,float *Z, int Offset){
  return sqrt(X[Offset]*X[Offset]+
	      Y[Offset]*Y[Offset]+
	      Z[Offset]*Z[Offset]);
  
}
void set_vector_normalized(float *X,float *Y,float *Z, int Offset){
  float len = vetor_length(X, Y, Z, Offset);
  if (len != 0){
    X[Offset] /= len; 
    Y[Offset] /= len; 
    Z[Offset] /= len; 
  }

}
