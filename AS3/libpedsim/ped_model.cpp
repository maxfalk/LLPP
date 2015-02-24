#include "ped_model.h"
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
    std::set<std::pair<Ped::Crowd*,int> > agents = tree->getAgents();
    for (auto it = agents.begin(); it != agents.end(); it++) {
      //doSafeMovementParallel(*it, tree);
    }
    collisionCheckNotDone[id] = false;
  }
  delete (collisionThreadData*) data;
}

void Ped::Model::setup(std::vector<Ped::Crowd*> crowdInScenario, 
		       IMPLEMENTATION _mode, int _nrOfThreads, bool _parallelCollision)
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
      
    treehash = new std::map< std::pair<Crowd*, int>, Ped::Ttree*>();
    //// Create a new quadtree containing all agents
    tree = new Ttree(NULL,treehash, 0, treeDepth, 0, 0, 1000, 800);
  
    //add all agents to tree
    for(int i = 0; i < crowds.size(); i++){
      for(int j = 0; j < crowds[i]->NumberOfAgents; j++){
	tree->addAgent(std::make_pair(crowds[i], j));
      }
    }
    for (int i = 0; i < COL_THREADS; i++) {
      collisionThreadData *data =  new collisionThreadData();
      data->id = i;
      //data->tree = trees[i]; //??
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
      doSafeMovment(std::make_pair(crowds[i],j));
      
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

void Ped::Model::doSafeMovment(std::pair<Ped::Crowd*, int> Agent){

//Search for neighboring agents
std::set<std::pair<Ped::Crowd*, int> > neighbors = 
  getNeighbors(Agent.first->AgentsX[Agent.second], 
	       Agent.first->AgentsY[Agent.second], 
	       2);

 std::vector<std::pair<int, int> > prioritizedAlternatives;
 std::pair<int,int> pDesired(Agent.first->DesiredX[Agent.second], 
			     Agent.first->DesiredY[Agent.second]);
  prioritizedAlternatives.push_back(pDesired);
  //Compute alternative ways of moving
  int diffX = pDesired.first - Agent.first->AgentsX[Agent.second];
  int diffY = pDesired.second - Agent.first->AgentsY[Agent.second];
  std::pair<int, int> p1, p2, p3,p4,p5,p6;


  p1 = std::make_pair(Agent.first->AgentsX[Agent.second] - diffX, 
		      Agent.first->AgentsY[Agent.second]);
  p2 = std::make_pair(Agent.first->AgentsX[Agent.second] + diffX, 
		      Agent.first->AgentsY[Agent.second]);
  p3 = std::make_pair(Agent.first->AgentsX[Agent.second], 
		      Agent.first->AgentsY[Agent.second] + diffY);
  p4 = std::make_pair(Agent.first->AgentsX[Agent.second], 
		      Agent.first->AgentsY[Agent.second] - diffY);


  prioritizedAlternatives.push_back(p1);
  prioritizedAlternatives.push_back(p2);
  prioritizedAlternatives.push_back(p3);
  prioritizedAlternatives.push_back(p4);

  //Find an empty spot of the once computed to move to
  for(int i=0; i < prioritizedAlternatives.size(); i++){
    bool taken = false;
    for(auto it = neighbors.begin(); it != neighbors.end(); ++it){
      
      if(it->first->AgentsX[it->second] == prioritizedAlternatives[i].first and
	 it->first->AgentsY[it->second] == prioritizedAlternatives[i].second){
	taken = true;
      }
    }
    
    if(taken == false){

      Agent.first->AgentsX[Agent.second] = prioritizedAlternatives[i].first;
      Agent.first->AgentsY[Agent.second] = prioritizedAlternatives[i].second;
      //Move agent in quadTree if necassary
      (*treehash)[Agent]->moveAgent(Agent);
      break;
    }    
  }
}
void Ped::Model::doSafeMovmentParallel(std::pair<Ped::Crowd*, int> Agent){


}
std::set<std::pair<Ped::Crowd*, int> > Ped::Model::getNeighbors(int x, int y, 
								int dist) const {
  // if there is no tree, return all agents
  if(tree == NULL) 
    return tree->getAgents();

  // create the output list
  std::list<std::pair<Crowd*, int> > neighborList;
  getNeighbors(neighborList, x, y, dist);

  // copy the neighbors to a set
  return std::set<std::pair<Crowd*, int> >(neighborList.begin(), neighborList.end());
}

void Ped::Model::getNeighbors(std::list<std::pair<Ped::Crowd*, int> >&neightborList, int x, 
			      int y, int dist) const{

  std::stack<Ped::Ttree*> treestack;

  treestack.push(tree);
  while(!treestack.empty()){
    Ped::Ttree *t = treestack.top();
    treestack.pop();
    if(t->isleaf){
      t->getAgents(neightborList);
    }
    else{
      if (t->tree1->intersects(x, y, dist)) treestack.push(t->tree1);
      if (t->tree2->intersects(x, y, dist)) treestack.push(t->tree2);
      if (t->tree3->intersects(x, y, dist)) treestack.push(t->tree3);
      if (t->tree4->intersects(x, y, dist)) treestack.push(t->tree4);
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
