#include "ped_model.h"
#include "cuda_dummy.h"
#include <pthread.h>
#include <stdlib.h>
#include <omp.h>
#include <iostream>


void Ped::Model::setup(std::vector<Ped::Crowd*> crowdInScenario, IMPLEMENTATION _mode, int _nrOfThreads)
{
  nrOfThreads = _nrOfThreads;
  crowds = crowdInScenario;
  implementation = _mode;
}

const std::vector<Ped::Crowd*> Ped::Model::getCrowds() const
{
  return crowds;
}

void *threaded_tick(void *para) {
  //Pthread here
}
void Ped::Model::seq()
{
  //Seq here
}
void Ped::Model::pThreads()
{
  //Pthreads here
}
void Ped::Model::omp()
{
  //OMP here
}
void Ped::Model::tick()
{
  if (implementation == SEQ) {
    seq();
  } else if (implementation == PTHREAD) {
    pThreads();
  }else if (implementation == VECTOR){
    //vector
  } else {
    omp();
  }
}

