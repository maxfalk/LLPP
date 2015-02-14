#include "ped_model.h"
#include "cuda_dummy.h"
#include <pthread.h>
#include <stdlib.h>
#include <omp.h>
#include <iostream>
#include <stdio.h>

std::vector<Ped::Crowd*> crowd;

void Ped::Model::setup(std::vector<Ped::Crowd*> crowdInScenario, IMPLEMENTATION _mode, int _nrOfThreads)
{
  nrOfThreads = _nrOfThreads;
  crowds = crowdInScenario;
  implementation = _mode;
  crowd = crowds;
}

const std::vector<Ped::Crowd*> Ped::Model::getCrowds() const
{
  return crowds;
}

void *threaded_tick(void *indices) {
    //Pthread here
}
void Ped::Model::seq()
{
  
  for(int i = 0; i < crowds.size(); i++){
    for(int j = 0; j < crowds[i]->NumberOfAgents; j++){
      crowds[i]->where_to_go(j);
      crowds[i]->go(j);
      
    }
   }

}
void Ped::Model::pThreads()
{
    //Pthreads here


}
void Ped::Model::omp()
{
  //OMP here
 
    for(int i = 0; i < crowds.size(); i++){
#pragma omp paralell
        { 
#pragma omp for
            for(int j = 0; j < crowds[i]->NumberOfAgents; j++){
                crowds[i]->where_to_go(j);
                crowds[i]->go(j);
            }
      
        }
    }


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

