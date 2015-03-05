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
    void cuda();

    static void doSafeMovement(Net::Npair Agent);
    static void doSafeMovementParallel(Net::Npair Agent);    
    static void *checkCollisions(void *data);
    void create_threads(int nrOfThreads);
    static void *threaded_tick(void *inds);

    
  };
  
  

}
#endif
