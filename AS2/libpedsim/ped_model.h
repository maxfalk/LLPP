#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include "ped_crowd.h"

namespace Ped{
  enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ};
  class Model
  {
  public:
    void setup(std::vector<Crowd*> crowdsInScenario, IMPLEMENTATION mode, 
	       int nrOfThreads);
    void tick();
    const std::vector<Crowd*> getCrowds() const;
  private:
    int nrOfThreads;
    IMPLEMENTATION implementation;
    std::vector<Crowd*> crowds;
    void omp();
    void seq();
    void pThreads();
  };
}
#endif
