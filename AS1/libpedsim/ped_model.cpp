#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <omp.h>
#include <iostream>
#include <xmmintrin.h>
#include <stdint.h>
#include <assert.h>

struct _threadInfo {
  int startIndex;
  int stopIndex;
  std::vector<Ped::Tagent*> *agents;
} typedef threadInfo;

void Ped::Model::setup(vector<Ped::Tagent*> agentsInScenario, IMPLEMENTATION _mode, int _nrOfThreads)
{
  nrOfThreads = _nrOfThreads;
  agents = agentsInScenario;
  implementation = _mode;
  
  int err;
  if(_mode == VECTOR){
    printf("preparing for vector\n");
    err = posix_memalign((void **)&agentPos, __alignof__(__m128d), 
		   agents.size()*2 * sizeof(double*));
    if(err != 0)
      printf("ERROR: out of mem\n");
    err = posix_memalign((void **)&agentForce, __alignof__(__m128d)
		   , agents.size()*2 * sizeof(double*));
    if(err != 0)
      printf("ERROR: out of mem\n");

    
    assert(0 == ((intptr_t)agentPos) % __alignof__(__m128d));
    assert(0 == ((intptr_t)agentForce) % __alignof__(__m128d));
   
    vecPrepare();
    printf("vector prepared\n");
  }

}

const std::vector<Ped::Tagent*> Ped::Model::getAgents() const
{
  return agents;
}

void *threaded_tick(void *para) {
  threadInfo *info = (threadInfo *) para;
  for (int i = info->startIndex; i < info->stopIndex; i++) {
    (*info->agents)[i]->whereToGo();
    (*info->agents)[i]->go();
  }
  delete info;

  return NULL;
}
void Ped::Model::seq()
{
  for (int i = 0; i < agents.size(); i++) {
    agents[i]->whereToGo();
    agents[i]->go();
  }
}
void Ped::Model::vecPrepare(){

  for(int i,a = 0; i < agents.size(); i++, a+=2){
    agentPos[a] = agents[i]->getPos()->x;
    agentPos[a+1] = agents[i]->getPos()->y;
    agentForce[a] = agents[i]->getForce()->x;
    agentForce[a+1] = agents[i]->getForce()->y;
  }

  printf("Num: %f\n", agentPos[0]);
  printf("Num1: %f\n", agentPos[1]);

}
void Ped::Model::vec()
{

  __m128d *m1 = (__m128d *)agentPos;
  __m128d *m2 = (__m128d *)agentForce;

  for(int i = 0; i < agents.size(); i++, agentPos += 2){
    agents[i]->whereToGo();
    //Go
    _mm_store_pd(agentPos, _mm_add_pd(*m1, *m2));
  }

}

void Ped::Model::pThreads()
{
  pthread_t threads[nrOfThreads];
 
  for (int i = 0; i < nrOfThreads; i++) {
    threadInfo *para = new threadInfo;
    para->startIndex = (agents.size()/nrOfThreads)*i;
    if (i == nrOfThreads-1)
      para->stopIndex = (agents.size());
    else
      para->stopIndex = (agents.size()/nrOfThreads)*(i+1);
    
    para->agents = &agents;
    pthread_create(&threads[i], NULL, threaded_tick, para);
  }

  for (int i = 0; i < nrOfThreads; i++) {
    pthread_join(threads[i], NULL);
  }

}
void Ped::Model::omp()
{
  omp_set_dynamic(0);
  omp_set_num_threads(nrOfThreads);
#pragma omp parallel
  {
#pragma omp for nowait
    for (int i = 0; i < agents.size(); i++) {
      agents[i]->whereToGo();
      agents[i]->go();
    }
  }
}
void Ped::Model::tick()
{
  if (implementation == SEQ) {
    seq();
  } else if (implementation == PTHREAD) {
    pThreads();
  } else if (implementation == VECTOR) {
    vec();
  } else {
    omp();
  }
}

