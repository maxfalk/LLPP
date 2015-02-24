#include "ped_model.h"
#include "ped_agent.h"
#include <pthread.h>
#include <stdlib.h>
#include <omp.h>
#include <iostream>
//#include <cuda_runtime.h>
#include <stdio.h>
#include <stack>
#include <algorithm>

std::vector<Ped::Crowd*> crowd;
bool dontKill;
pthread_t collisionThreads[COL_THREADS];
bool noCollisionCheck[COL_THREADS];
bool collisionCheckNotDone[COL_THREADS];

typedef struct _collisionThreadData {
  int id;
  Ped::Ttree *tree;
} collisionThreadData;

void *checkCollisions(void *data) {
  int id = ((collisionThreadData*) data)->id;
  Ped::Ttree *tree = ((collisionThreadData*) data)->tree;
  while (dontKill) {
    while (noCollisionCheck[id]) {}
    noCollisionCheck[id] = true;
    std::set<pair<Ped::Crowd*,int> > agents = tree->getAgents();
    for (auto it = agents.begin(); it != agents.end(); it++) {
      doSafeMovementParallel(*it, tree);
    }
    collisionCheckNotDone[id] = false;
  }
  delete (collisionThreadData*) data;
}

void Ped::Model::setup(std::vector<Ped::Crowd*> crowdInScenario, IMPLEMENTATION _mode, int _nrOfThreads, bool _parallelCollision)
{
    nrOfThreads = _nrOfThreads;
    crowds = crowdInScenario;
    implementation = _mode;
    parallelCollision = _parallelCollision;
    crowd = crowds;
    dontKill = true;
    for (int i = 0; i < COL_THREADS; i++) {
      noCollisionCheck[i] = true;
      collisionCheckNotDone[i] = true;
    }

    //if(implementation == CUDA){
    //    for(int i = 0; i < crowds.size(); i++)
    //      crowds[i]->init_cuda();
    //}
    
    //// Hack! do not allow agents to be on the same position. Remove duplicates from scenario.
    //bool (*fn_pt)(Ped::Tagent*, Ped::Tagent*) = cmp;
    //std::set<Ped::Tagent*, bool(*)(Ped::Tagent*, Ped::Tagent*)> agentsWithUniquePosition (fn_pt);
    //std::copy(agentsInScenario->begin(), agentsInScenario->end(), std::inserter(agentsWithUniquePosition, agentsWithUniquePosition.begin()));

    //agents = std::vector<Ped::Tagent*>(agentsWithUniquePosition.begin(), agentsWithUniquePosition.end());
    //treehash = new std::map<const Ped::Tagent*, Ped::Ttree*>();

    //// Create a new quadtree containing all agents
    //tree = new Ttree(NULL,treehash, 0, treeDepth, 0, 0, 1000, 800);

    //for (std::vector<Ped::Tagent*>::iterator it = agents.begin(); it != agents.end(); ++it)
    //{
    //std::cout << (*it)->getX() << " " << (*it)->getY() << std::endl;
    //tree->addAgent(*it);
    //}
    for (int i = 0; i < COL_THREADS; i++) {
      collisionThreadData *data =  new collisionThreadData();
      data->id = i;
      data->tree = trees[i]; //??
      pthread_create(&collisionThreads[i], NULL, checkCollisions, (void*) data);
    }
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
    for (int i = 0; i < crowd.size(); i++) {
      for (int j = startIndex; j < stopIndex; j++) {
        crowd[i]->where_to_go(j);
        crowd[i]->go(j);
      }
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
void create_threads(int nrOfThreads) {
  pthread_t threads[nrOfThreads];
  for (int i=0; i<nrOfThreads; i++) {
    int *indices = (int *) malloc(2*sizeof(int));
    indices[0] = (crowd[0]->NumberOfAgents/nrOfThreads)*i;
    if (nrOfThreads-i == 1) {
      indices[1] = crowd[0]->NumberOfAgents;
    } else {
      indices[1] = (crowd[0]->NumberOfAgents/nrOfThreads)*(i+1);
    }
    pthread_create(&threads[i], NULL, threaded_tick, (void *) indices);
  }
  for (int i = 0; i < nrOfThreads; i++) {
    pthread_join(threads[i], NULL);
  }
}
void Ped::Model::pThreads()
{
  create_threads(nrOfThreads);
  
}
void Ped::Model::omp()
{
  omp_set_dynamic(0);
  omp_set_num_threads(nrOfThreads);
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
void Ped::Model::vector()
{

  for(int i = 0; i < crowds.size(); i++){
    for(int j = 0; j < crowds[i]->NumberOfAgents; j+=4){
      crowds[i]->where_to_go_vectorized(j);
      crowds[i]->go_vectorized(j);
      
    }
   }

}
//void Ped::Model::cuda(){
//
//  
//  for(int i = 0; i < crowds.size(); i++){
//    crowds[i]->where_to_go_cuda();
//    crowds[i]->go_cuda();
//  }
//
//
//}
void Ped::Model::tick()
{
  if (implementation == SEQ) {
    seq();
  } else if (implementation == PTHREAD) {
    pThreads();
  }else if (implementation == VECTOR){
    vector();
  //}else if(implementation == CUDA){ 
  //  cuda();
  }else {
    omp();
  }
  if (parallelCollision) {
    for (int i = 0; i < COL_THREADS; i++) {
      noCollisionCheck[i] = false;
    }
    for (int i = 0; i < COL_THREADS; i++) {
      while(collisionCheckNotDone[i]) {}
      collisionCheckNotDone[i] = true;
    }
  }
}

void Ped::Model::cleanup() {
  if (parallelCollision) {
    dontKill = false;
    for (int i = 0; i < COL_THREADS; i++) {
      pthread_join(collisionThreads[i], NULL);
    }
  }
}
