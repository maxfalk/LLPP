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

void *threaded_tick(void *inds) {
    //Pthread here
    int *indices = (int *) inds;
    int startIndex = indices[0];
    int stopIndex = indices[1];
    for (int i=startIndex; i<stopIndex; i++) {
        crowd[0]->where_to_go(i);
        crowd[0]->go(i);
        crowd[1]->where_to_go(i);
        crowd[1]->go(i);
    }
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
void create_threads(Ped::Crowd* c, int nrOfThreads) {
    pthread_t threads[nrOfThreads];
    for (int i=0; i<nrOfThreads; i++) {
        int *indices = (int *) malloc(2*sizeof(int));
        indices[0] = (c->NumberOfAgents/nrOfThreads)*i;
        if (nrOfThreads-i == 1) {
            indices[1] = c->NumberOfAgents;
        } else {
            indices[1] = (c->NumberOfAgents/nrOfThreads)*(i+1);
        }
        pthread_create(&threads[i], NULL, threaded_tick, (void *) indices);
    }
}
void Ped::Model::pThreads()
{

    create_threads(crowds[0], nrOfThreads);
    create_threads(crowds[1], nrOfThreads);

}
void Ped::Model::omp()
{
    //OMP here
    #pragma omp parallel for
    for(int i = 0; i < crowds[0]->NumberOfAgents; i++){
        crowds[0]->where_to_go(i);
        crowds[0]->go(i);
    }
    #pragma omp parallel for
    for(int i = 0; i < crowds[1]->NumberOfAgents; i++){
        crowds[1]->where_to_go(i);
        crowds[1]->go(i);
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

