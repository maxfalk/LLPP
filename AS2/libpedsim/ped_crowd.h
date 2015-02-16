#ifndef _ped_crowd_h_
#define _ped_crowd_h_
#include <xmmintrin.h>

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
    void constructor();
    void init();
    void go(int Offset);
    void where_to_go(int Offset);
    Crowd(int, int, int);
    ~Crowd();
    //Vectorized methods
    void go_vectorized(int);
    void where_to_go_vectorized(int Offset);
    //Cuda
    void cuda_free();
    void init_cuda();
    void go_cuda();
    void where_to_go_cuda();

  private:
    //Variables
    int mode;
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

    int NumberOfWaypoints;
    int *CurrentWaypoint;

    //Methods
    void vector_alloc(void **, int);
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
  
    //Vectorized
    void constructor_vector();
    void vector_length_vectorized(__m128*,__m128*,__m128*, __m128*);
    void set_vector_normalized_vectorized(__m128*,__m128*,__m128*,__m128*);
    void set_force_vectorized(__m128 *mTempX, __m128 *mTempY, float *mTempZ, 
			      __m128 *X, __m128 *Y, int Offset, bool *reached);
    void computeDirection_vectorized(int Offset);
    void setNextDestination_vectorized(int Offset);


    //cuda, device pointers
    float *d_AgentsX;
    float *d_AgentsY;
    float *d_MoveForceX;
    float *d_MoveForceY;
    float *d_DestX;
    float *d_DestY;
    float *d_DestR;
    float *d_LastDestX;
    float *d_LastDestY;
    float *d_LastDestR;
    float *d_CurrWay;
 
    float *d_WaypointX;
    float *d_WaypointY;
    float *d_WaypointR;
    float *d_NumberOfWaypoints;



  };
};
#endif
