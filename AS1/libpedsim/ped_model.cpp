#include "ped_model.h"
#include "ped_waypoint.h"
#include "cuda_dummy.h"
#include <pthread.h>
#include <stdlib.h>
#include <omp.h>
#include <iostream>

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
  } else {
    omp();
  }
}

