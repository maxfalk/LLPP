#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include <map>
#include "ped_crowd.h"
#include "ped_net.h"
#define COL_THREADS 4

namespace Ped{

  enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ};
  class Model
  {
  public:
    void setup(std::vector<Crowd*> crowdsInScenario, IMPLEMENTATION mode, 
	       int nrOfThreads, bool parallelCollision);
    void tick();
    const std::vector<Crowd*> getCrowds() const;

    void cleanup();
    //~Model();
  private:
    int nrOfThreads;
    IMPLEMENTATION implementation;
    std::vector<Crowd*> crowds;
    bool parallelCollision;
    void omp();
    void seq();
    void pThreads();
    void vector();
    //void cuda();
   

    void doSafeMovment(std::pair<Ped::Crowd*, int> *Agent);
    // The maximum quadtree depth
    static const int treeDepth = 10;    


    static void doSafeMovementParallel(std::pair<Ped::Crowd*, int> *Agent);
    
    static void *checkCollisions(void *data);
    
  
    
  };
}
#endif
