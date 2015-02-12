#ifndef _ped_crowd_h_
#define _ped_crowd_h_

namespace Ped{
  class Crowd {
  public:
    int NumberOfAgents;
    //Agent pos
    float *AgentsX;
    float *AgentsY;
    float *AgentsZ;
    //MoveForce per agent arrays of agents
    //correspond to the same pos for moveforce
    float *MoveForceX;
    float *MoveForceY;
    float *MoveForceZ;
   
    //Waypoints, all the waypoints agents go between.
    float *WaypointX;
    float *WaypointY;
    float *WaypointR;
   
    
    //Methods
    void init();
    void go(int Offset);
    void where_to_go(int Offset);
    Crowd(int, int);
    ~Crowd();

  private:
    //Variables
    //Ped::Implementation mode;
    //Destination for each agent
    //Arrays
    float *DestinationX;
    float *DestinationY;
    float *DestinationR;
    
    //Last destination for each agent
    //Arrays
    float *LastDestinationX;
    float *LastDestinationY;
    float *LastDestinationR;

    //Number of waypoints, eg length(WaypointX) which is the same length as 
    // length(WaypointY) and length(WaypointR)
    int NumberOfWaypoints;
    int CurrentWaypoint;

    //Methods
    void vector_alloc(void **);
    void setNextDestination(int Offset);
    void computeDirection(int Offset);
    void set_force(float *temp, float *X, float *Y,
		   int Offset, bool *reached);
    float vector_dotproduct(int Agent1, int Agent2);
    float *vector_subtract(int Agent1, int Agent2);
    float vector_length(float *X,float *Y,float *Z, int Offset);
    float vector_length(float X,float Y,float Z);
    void set_vector_normalized(float *X,float *Y,float *Z, int Offset);
    void set_vector_normalized(float *X,float *Y,float *Z, 
			       int Offset, float len);
    


  };
};
#endif
