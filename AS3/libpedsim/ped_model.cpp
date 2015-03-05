#include "ped_model.h"
#include "ped_net.h"
#include <pthread.h>
#include <stdlib.h>
#include <omp.h>
#include <iostream>
#include <stdio.h>
#include <stack>
#include <algorithm>
#include <assert.h>

std::vector<Ped::Crowd*> crowd;

int startIndexX[COL_THREADS];
int stopIndexX[COL_THREADS];
int startIndexY[COL_THREADS];
int stopIndexY[COL_THREADS];
int sumPosX[COL_THREADS];
int sumPosY[COL_THREADS];
int totalAgents;
int threadsWork[COL_THREADS];
Ped::Net *net;

void Ped::Model::setup(std::vector<Ped::Crowd*> crowdInScenario, 
		       IMPLEMENTATION _mode, int _nrOfThreads, bool _parallelCollision) {

    nrOfThreads = _nrOfThreads;
    crowds = crowdInScenario;
    implementation = _mode;
    parallelCollision = _parallelCollision;
    crowd = crowds;
    net = new Net(200,200);

    if(implementation == CUDA){
      for(int i = 0; i < crowds.size(); i++){
	//crowds[i]->init_cuda();
      }
    }
    

    totalAgents = 0;
    for (int i = 0; i < crowds.size(); i++) {
      Net::Npair par = (Net::Npair) malloc(sizeof(Net::_Npair) * crowds[i]->NumberOfAgents);  	 
      totalAgents += crowds[i]->NumberOfAgents;
      for (int j = 0; j < crowds[i]->NumberOfAgents; j++) {
	  par[j].first = crowds[i];
	  par[j].second = j;
	  net->field[(int)crowds[i]->AgentsX[j]][(int)crowds[i]->AgentsY[j]] = &par[j];
	 
        }
    }

    if(parallelCollision){
      startIndexX[0] = 0;
      stopIndexX[0] = (net->sizeX)/2;
      startIndexY[0] = 0;
      stopIndexY[0] = (net->sizeY)/2;
      startIndexX[1] = (net->sizeX)/2;
      stopIndexX[1] = net->sizeY;
      startIndexY[1] = 0;
      stopIndexY[1] = (net->sizeY)/2;
      startIndexX[2] = 0;
      stopIndexX[2] = (net->sizeY)/2;
      startIndexY[2] = (net->sizeX)/2;
      stopIndexY[2] = net->sizeY;
      startIndexX[3] = (net->sizeX)/2;
      stopIndexX[3] = net->sizeY;
      startIndexY[3] = (net->sizeX)/2;
      stopIndexY[3] = net->sizeY;
    }


}
const std::vector<Ped::Crowd*> Ped::Model::getCrowds() const
{
    return crowds;
}

void *Ped::Model::threaded_tick(void *inds) {
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
void Ped::Model::create_threads(int nrOfThreads) {
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
void Ped::Model::cuda(){

  
  for(int i = 0; i < crowds.size(); i++){
    //crowds[i]->where_to_go_cuda();
    //crowds[i]->go_cuda();
  }


}
void Ped::Model::tick()
{

  if (implementation == SEQ) {
    seq();
  } else if (implementation == PTHREAD) {
    pThreads();
  }else if (implementation == VECTOR){
    vector();
  }else if(implementation == CUDA){ 
    cuda();
  }else {
    omp();
  }


  if (parallelCollision) {
    //Run threads
    pthread_t collisionThreads[COL_THREADS];
    int *id = new int[COL_THREADS];
    for (int i = 0; i < COL_THREADS; i++) {
      id[i] = i;
      pthread_create(&collisionThreads[i], NULL, Ped::Model::checkCollisions, (void*) &id[i]);
    }
    for (int i = 0; i < COL_THREADS; i++) {
      pthread_join(collisionThreads[i], NULL);
    }

    delete[] id;
    //Naive load balance
    //JIT load blancing
    int sumX = 0;
    int sumY = 0; 
    for (int i=0; i<COL_THREADS; i++) {
      sumX += sumPosX[i];
      sumY += sumPosY[i];
    }
    int avgX = sumX/totalAgents; 
    int avgY = sumY/totalAgents; 
    stopIndexX[0] = avgX;
    stopIndexY[0] = avgY;
    startIndexX[1] = avgX;
    stopIndexY[1] = avgY;
    stopIndexX[2] = avgX;
    startIndexY[2] = avgY;
    startIndexX[3] = avgX;
    startIndexY[3] = avgY;
  }else{

    Net::_Npair par;
    for(int i = 0; i < crowds.size(); i++){
      for(int j = 0; j < crowds[i]->NumberOfAgents; j++){
	par.first = crowds[i];
	par.second = j;
	doSafeMovement(&par);      
      }
    }
  }
}

void Ped::Model::doSafeMovement(Net::Npair Agent){


 std::vector<std::pair<int, int> > prioritizedAlternatives;
 std::pair<int,int> pDesired(Agent->first->DesiredX[Agent->second], 
			     Agent->first->DesiredY[Agent->second]);
  prioritizedAlternatives.push_back(pDesired);
  //Compute alternative ways of moving
  int diffX = pDesired.first - Agent->first->AgentsX[Agent->second];
  int diffY = pDesired.second - Agent->first->AgentsY[Agent->second];
  std::pair<int, int> p1, p2;

  if (diffX == 0 || diffY == 0)
    {
      // Agent wants to walk straight to North, South, West or East
      p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
      p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
    }
  else {
    // Agent wants to walk diagonally
    p1 = std::make_pair(pDesired.first, Agent->first->AgentsY[Agent->second]);
    p2 = std::make_pair(Agent->first->AgentsX[Agent->second], pDesired.second);
  }
  prioritizedAlternatives.push_back(p1);
  prioritizedAlternatives.push_back(p2);


  //Find an empty spot of the once computed to move to
  for(int i=0; i < prioritizedAlternatives.size(); i++){
    bool taken = false;
    if(prioritizedAlternatives[i].first >= 0 && 
       prioritizedAlternatives[i].second >= 0 &&
       prioritizedAlternatives[i].first < net->sizeX && 
       prioritizedAlternatives[i].second <  net->sizeY){
      
    if(net->field[prioritizedAlternatives[i].first][prioritizedAlternatives[i].second] != NULL){
      taken = true;
    }
    
    if(taken == false){
        net->field[prioritizedAlternatives[i].first][prioritizedAlternatives[i].second] = Agent;
        net->field[(int)Agent->first->AgentsX[Agent->second]][(int)Agent->first->AgentsY[Agent->second]]=NULL;
        Agent->first->AgentsX[Agent->second] = prioritizedAlternatives[i].first;
        Agent->first->AgentsY[Agent->second] = prioritizedAlternatives[i].second;
        break;
    } 
    }   
  }

}
void Ped::Model::doSafeMovementParallel(Net::Npair Agent) {

  std::vector<std::pair<int, int> > prioritizedAlternatives;
  std::pair<int,int> pDesired(Agent->first->DesiredX[Agent->second], 
			      Agent->first->DesiredY[Agent->second]);
  prioritizedAlternatives.push_back(pDesired);
  //Compute alternative ways of moving
  int diffX = pDesired.first - Agent->first->AgentsX[Agent->second];
  int diffY = pDesired.second - Agent->first->AgentsY[Agent->second];
  std::pair<int, int> p1, p2;

  if (diffX == 0 || diffY == 0)
    {
      // Agent wants to walk straight to North, South, West or East
      p1 = std::make_pair(pDesired.first + diffY, pDesired.second + diffX);
      p2 = std::make_pair(pDesired.first - diffY, pDesired.second - diffX);
    }
  else {
    // Agent wants to walk diagonally
    p1 = std::make_pair(pDesired.first, Agent->first->AgentsY[Agent->second]);
    p2 = std::make_pair(Agent->first->AgentsX[Agent->second], pDesired.second);
  }
  prioritizedAlternatives.push_back(p1);
  prioritizedAlternatives.push_back(p2);


 //Find an empty spot of the once computed to move to
  for(int i=0; i < prioritizedAlternatives.size(); i++){
    if(prioritizedAlternatives[i].first >= 0 && 
       prioritizedAlternatives[i].second >= 0 &&
       prioritizedAlternatives[i].first < net->sizeX && 
       prioritizedAlternatives[i].second <  net->sizeY){
      
      bool successfulSwap = net->take_pos_atomic(prioritizedAlternatives[i].first, 
					       prioritizedAlternatives[i].second, Agent);
      if (successfulSwap) {
	int x = Agent->first->AgentsX[Agent->second];
	int y = Agent->first->AgentsY[Agent->second];
	Agent->first->AgentsX[Agent->second] = prioritizedAlternatives[i].first;
	Agent->first->AgentsY[Agent->second] = prioritizedAlternatives[i].second;
	net->field[x][y] = NULL;
	break;  
      }
    }
  }    
}
void *Ped::Model::checkCollisions(void *data) {

  int id = *((int*) data);
  sumPosX[id] = 0;
  sumPosY[id] = 0;
  threadsWork[id] = 0; 
  for (int i = startIndexX[id]; i < stopIndexX[id]; i++) {
    for (int j = startIndexY[id]; j < stopIndexY[id]; j++) {
      Net::Npair Agent = net->field[j][i];
      if(Agent != NULL){
        threadsWork[id]++; 
        sumPosX[id] += Agent->first->AgentsX[Agent->second];
        sumPosY[id] += Agent->first->AgentsY[Agent->second];
        if(Agent->first->AgentsX[Agent->second] >= (stopIndexX[id]-1) or
	   Agent->first->AgentsX[Agent->second] <= (startIndexX[id]-1) or
	   Agent->first->AgentsY[Agent->second] >= (stopIndexY[id]-1) or
	   Agent->first->AgentsY[Agent->second] <= (startIndexY[id]-1)){
	  
	  doSafeMovementParallel(Agent);
	}else{
	  doSafeMovement(Agent);
	}
      }
    }
  }
  
}
