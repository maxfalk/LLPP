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
volatile bool dontKill;
pthread_t collisionThreads[COL_THREADS];
volatile bool noCollisionCheck[COL_THREADS];
volatile bool collisionCheckNotDone[COL_THREADS];
Ped::Net *net;
  

typedef struct{
  int id;
  int start;
  int end;
} _collisiondata, *collisiondata;

void Ped::Model::setup(std::vector<Ped::Crowd*> crowdInScenario, 
		       IMPLEMENTATION _mode, int _nrOfThreads, bool _parallelCollision) {

    nrOfThreads = _nrOfThreads;
    crowds = crowdInScenario;
    implementation = _mode;
    parallelCollision = _parallelCollision;
    crowd = crowds;
    dontKill = true;
    net = new Net(1000,800);
    for (int i = 0; i < COL_THREADS; i++) {
        noCollisionCheck[i] = true;
        collisionCheckNotDone[i] = true;
    }

    if(implementation == CUDA){
      for(int i = 0; i < crowds.size(); i++){
	//crowds[i]->init_cuda();
      }
    }
    

    for (int i = 0; i < crowds.size(); i++) {
      Net::Npair par = (Net::Npair) malloc(sizeof(Net::_Npair) * crowds[i]->NumberOfAgents);  	 
      for (int j = 0; j < crowds[i]->NumberOfAgents; j++) {
	  par[j].first = crowds[i];
	  par[j].second = j;
	  net->field[(int)crowds[i]->AgentsX[j]][(int)crowds[i]->AgentsY[j]] = &par[j];
	 
        }
    }


    if (parallelCollision == true){
      for (int i = 0; i < COL_THREADS; i++) {
	collisiondata data = (collisiondata) malloc(sizeof(_collisiondata));
	data->id = i;
	data->start = (net->sizeY/COL_THREADS)*i;
	data->end = (net->sizeY/COL_THREADS)*(i+1);
	pthread_create(&collisionThreads[i], NULL, Ped::Model::checkCollisions, (void*) data);
      }
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
    for (int i = 0; i < COL_THREADS; i++) {
      noCollisionCheck[i] = false;
    }
    for (int i = 0; i < COL_THREADS; i++) {
      while(collisionCheckNotDone[i]) {}
      collisionCheckNotDone[i] = true;
    }
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
  // int totalOfAgents = 0;
  // int actualAgents = 0;
  // for(int i = 0; i < crowds.size(); i++){
  //   totalOfAgents += crowds[i]->NumberOfAgents;
  //   }
  //    omp_set_dynamic(0);
  //   omp_set_num_threads(nrOfThreads);
  // //OMP here
    
  //   for (int i = 0; i < net->sizeX; i++) {
  //       #pragma omp parallel for reduction (+:actualAgents)
  //       for (int j = 0; j < net->sizeY; j++) {
  //           if (net->field[i][j] != NULL) {
  //               actualAgents++;
  //           }
  //       }
  //   }
    
  //   assert(totalOfAgents == actualAgents);




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

  int startIndex = ((collisiondata)data)->start;
  int stopIndex = ((collisiondata)data)->end;
  int id = ((collisiondata)data)->id;
  printf("start %d: %d\n", id, startIndex);
  printf("stop %d: %d\n", id, stopIndex);
  while (dontKill) {
    while (noCollisionCheck[id] == true) {}
    noCollisionCheck[id] = true;
    for (int i = startIndex; i < stopIndex; i++) {
      for (int j = 0; j < net->sizeX; j++) {
	Net::Npair Agent = net->field[j][i];
	if(Agent != NULL){
	  if(Agent->first->AgentsY[Agent->second] >= stopIndex-1 or
	     Agent->first->AgentsY[Agent->second] <= startIndex-1){
	    
	    doSafeMovementParallel(Agent);
	  }else{
	    doSafeMovement(Agent);
	  }

	}
      }
    }
    collisionCheckNotDone[id] = false;
  }
  free(data);
}

void Ped::Model::cleanup() {
  if (parallelCollision) {
    dontKill = false;
    for (int i = 0; i < COL_THREADS; i++) {
      noCollisionCheck[i] = false;
    }

    for (int i = 0; i < COL_THREADS; i++) {
      pthread_join(collisionThreads[i], NULL);
    }
  }
}
