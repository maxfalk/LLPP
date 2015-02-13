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
    int startIndex = *(int *) indices;
    int stopIndex = *(int *) (indices)+1;
    for(int i = 0; startIndex<stopIndex; i++){
        crowd[i]->where_to_go(i);
        crowd[i]->go(i);
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
void Ped::Model::pThreads()
{
    //Pthreads here
    pthread_t threads[nrOfThreads];
    Crowd *crowd1 = crowds[0];
    Crowd *crowd2 = crowds[1];

    for (int i=0; i<nrOfThreads; i++) {
        int *indices = (int *) malloc(2*sizeof(int));
        indices[0] = (crowd1->NumberOfAgents/nrOfThreads)*i;
        if (i == nrOfThreads-1) {
            indices[1] = crowd1->NumberOfAgents;
        } else {
            indices[1] = (crowd1->NumberOfAgents/nrOfThreads)*(i+1);
        }
        pthread_create(&threads[i], NULL, threaded_tick, (void *) indices);
    }
    for (int i=0; i<nrOfThreads; i++) {
        int *indices = (int *) malloc(2*sizeof(int));
        indices[0] = (crowd2->NumberOfAgents/nrOfThreads)*i; 
        if (i == nrOfThreads-1) {
            indices[1] = crowd2->NumberOfAgents;
        } else {
            indices[1] = (crowd2->NumberOfAgents/nrOfThreads)*(i+1);
        }
        pthread_create(&threads[i], NULL, threaded_tick, (void *) indices);
    }
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

