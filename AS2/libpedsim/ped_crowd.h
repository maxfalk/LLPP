#ifndef _ped_crowd_h_
#define _ped_crowd_h_

namespace Ped{
  class Crowd {
  public:
    //Agent pos
    float *AgentsX;
    float *AgentsY;
    float *AgentsZ;
    //MoveForce per agent arrays of agents
    //correspond to the same pos for moveforce
    float *MoveForceX;
    float *MoveForceY;
    float *MoveForceZ;
    
    void go();
    void where_to_go();
  private:
    //Destination for each agent
    //Arrays
    int *DestinationX;
    int *DestinationY;
    int *DestinationR;
    
    //Last destination for each agent
    //Arrays
    int *LastDestinationX;
    int *LastDestinationY;
    int *LastDestinationR;

    //Waypoints, all the waypoints agents go between.
    int *WaypointX;
    int *WaypointY;
    int *WaypointR;
    //Number of waypoints, eg length(WaypointX) which is the same length as 
    // length(WaypointY) and length(WaypointR)
    int NumberOfWaypoints;

    
  }
}
#endif
