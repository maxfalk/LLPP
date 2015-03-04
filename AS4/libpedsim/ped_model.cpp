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
#include <cuda_runtime.h>
#include "heat_cuda.h"
#define MAXCUDATHREADS 512

std::vector<Ped::Crowd*> crowd;
volatile bool dontKill;
pthread_t collisionThreads[COL_THREADS];
volatile bool noCollisionCheck[COL_THREADS];
volatile bool collisionCheckNotDone[COL_THREADS];
int ThreadsWork[COL_THREADS];
int startIndexX[COL_THREADS];
int stopIndexX[COL_THREADS];
int startIndexY[COL_THREADS];
int stopIndexY[COL_THREADS];
int balanceSpeed[COL_THREADS];
Ped::Net *net;

//static
int *tempDesY;
int *tempDesX;
int *d_DesiredX;
int *d_DesiredY;
int *d_scaled_heatmap;
int *d_heatmap;
int *d_blurred_heatmap;

int **Ped::Model::blurred_heatmap;

void Ped::Model::setup(std::vector<Ped::Crowd*> crowdInScenario, 
		       IMPLEMENTATION _mode, int _nrOfThreads, 
		       bool _parallelCollision, bool heatMap) {

    nrOfThreads = _nrOfThreads;
    crowds = crowdInScenario;
    implementation = _mode;
    parallelCollision = _parallelCollision;
    crowd = crowds;
    dontKill = true;
    net = new Net(400,400);
    heatmapPar = heatMap;

    //setup heatmap
    if(heatmapPar == true){
      start_setup_heat_map();
    }else{
      setupHeatmapSeq();
    }
      //setup CUDA
    if(implementation == CUDA){
      for(int i = 0; i < crowds.size(); i++){
	crowds[i]->init_cuda();
      }
    }
    
    //Setup net
    for (int i = 0; i < crowds.size(); i++) {
      Net::Npair par = (Net::Npair) malloc(sizeof(Net::_Npair) * 
					   crowds[i]->NumberOfAgents);  	 
      for (int j = 0; j < crowds[i]->NumberOfAgents; j++) {
	  par[j].first = crowds[i];
	  par[j].second = j;
	  net->field[(int)crowds[i]->AgentsX[j]][(int)crowds[i]->AgentsY[j]] = &par[j];
	 
        }
    }

    //Collision
    if (parallelCollision == true){
      for (int i = 0; i < COL_THREADS; i++) {
        noCollisionCheck[i] = true;
        collisionCheckNotDone[i] = true;
      }

      int *id = new int[COL_THREADS];
      for (int i = 0; i < COL_THREADS; i++) {
	balanceSpeed[i] = 1;
	id[i] = i;
	pthread_create(&collisionThreads[i], NULL, 
		       Ped::Model::checkCollisions, (void*) &id[i]);
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
    int i = indices[2];
    int startIndex = indices[0];
    int stopIndex = indices[1];
    for (int j = startIndex; j < stopIndex; j++) {
      crowd[i]->where_to_go(j);
      crowd[i]->go(j);
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
  int crowdSize = crowds.size();
  int split = nrOfThreads / crowdSize;
  int rest = nrOfThreads % crowdSize;
  int NumThreads[crowdSize];
  for(int i = 0; i < crowdSize; i++){
    NumThreads[i] = split;
    if(rest > 0){
      NumThreads[i]++;
      rest--;
    }
  }
 
  int thread = 0;
  for(int i = 0; i < crowdSize; i++){
    for(int j = 0; j < NumThreads[i]; j++){
      int *indices = (int *) malloc(3*sizeof(int));
      indices[2] = i;
      indices[0] = (crowd[i]->NumberOfAgents/NumThreads[i])*j;
      if (j == NumThreads[j]-1) {
	indices[1] = crowd[i]->NumberOfAgents;
      }else {
	indices[1] = (crowd[i]->NumberOfAgents/NumThreads[i])*(j+1);
      }
      pthread_create(&threads[thread], NULL, threaded_tick, (void *) indices);
      thread++;   
    }
    
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
    crowds[i]->where_to_go_cuda();
    crowds[i]->go_cuda();
  }


}
void Ped::Model::assertAgents(){
  //Assert that no two agents have the sam position on the map
  int totalOfAgents = 0;
  int actualAgents = 0;
  for(int i = 0; i < crowds.size(); i++){
    totalOfAgents += crowds[i]->NumberOfAgents;
  }
  omp_set_dynamic(0);
  omp_set_num_threads(nrOfThreads);
  //OMP here
  
  for (int i = 0; i < net->sizeX; i++) {
#pragma omp parallel for reduction (+:actualAgents)
    for (int j = 0; j < net->sizeY; j++) {
      if (net->field[i][j] != NULL) {
	actualAgents++;
      }
    }
  }
     
  assert(totalOfAgents == actualAgents);


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

  //Start Heatmap
  pthread_t thread;  
  if(heatmapPar == true){
    pthread_create(&thread, NULL, updateHeatmapPar , (void *) 0);
  }else{
    updateHeatmapSeq();
  }


  //Do Collision check
  if (parallelCollision) {
    //Run threads
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

  //Wait for heatmap to finish, atleast the starter thread
  if(heatmapPar == true){
    pthread_join(thread, NULL);
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
    int priox = prioritizedAlternatives[i].first;
    int prioy = prioritizedAlternatives[i].second;
    if(net->inbound(priox,prioy)){
      
      if(net->field[priox][prioy] != NULL){
	taken = true;
      }
      
      if(taken == false){
        net->field[priox][prioy] = Agent;
	int x = (int)Agent->first->AgentsX[Agent->second];
	int y = (int)Agent->first->AgentsY[Agent->second];
        net->field[x][y] = NULL;
        Agent->first->AgentsX[Agent->second] = priox;
        Agent->first->AgentsY[Agent->second] = prioy;
	
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
    int priox = prioritizedAlternatives[i].first;
    int prioy = prioritizedAlternatives[i].second;

    if(net->inbound(priox, prioy)){
      
      bool successfulSwap = net->take_pos_atomic(priox, prioy, Agent);
      if (successfulSwap) {
	int x = Agent->first->AgentsX[Agent->second];
	int y = Agent->first->AgentsY[Agent->second];
	Agent->first->AgentsX[Agent->second] = priox;
	Agent->first->AgentsY[Agent->second] = prioy;
	net->field[x][y] = NULL;
	break;  
      }
    }
  }    
}
bool on_thread_border(int x, int y, int id){
  return (x >= (stopIndexX[id]-1) or
	  x <= (startIndexX[id]-1) or
	  y >= (stopIndexY[id]-1) or
	  y <= (startIndexY[id]-1));

}
void *Ped::Model::checkCollisions(void *data) {

  int id = *((int*) data);
  startIndexX[id] = (net->sizeX/2)*(id%2);
  stopIndexX[id] = (net->sizeX/2)*((id%2)+1);
  startIndexY[id] = (net->sizeY/2)*(id%2);
  stopIndexY[id] = (net->sizeY/2)*((id%2)+1);
  while (dontKill) {
    while (noCollisionCheck[id] == true) {}
    noCollisionCheck[id] = true;
    ThreadsWork[id] = 0;
    //printf("start %d: %d\n", id, startIndex[id]);
    //printf("stop %d: %d\n", id, stopIndex[id]);

    for (int i = startIndexX[id]; i < stopIndexX[id]; i++) {
      for (int j = startIndexY[id]; j < stopIndexY[id]; j++) {
	Net::Npair Agent = net->field[j][i];
	if(Agent != NULL){
	  ThreadsWork[id]++;
	  int x = Agent->first->AgentsX[Agent->second];
	  int y = Agent->first->AgentsY[Agent->second];
	  if(on_thread_border(x, y, id)){
	    doSafeMovementParallel(Agent);
	  }else{
	    doSafeMovement(Agent);
	  } 
	}
      }
    }
    collisionCheckNotDone[id] = false;
  }
  if(id == 0)
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
void Ped::Model::calc_max_cuda_threads(int size){

  cudaThreads = MAXCUDATHREADS;
  while(size % cudaThreads != 0){
    cudaThreads--;
  }


}

void Ped::Model::start_setup_heat_map(){
  printf("Starting steup heat\n");

  blurred_heatmap = (int**) malloc(SCALED_SIZE*sizeof(int*));
  int *bhm = (int*) malloc(SCALED_SIZE*SCALED_SIZE*sizeof(int));
  
  for(int i = 0; i < SCALED_SIZE; i++){
    blurred_heatmap[i] = bhm + SCALED_SIZE*i;
  }

  
  cudaMalloc(&d_heatmap, SIZE*SIZE*sizeof(int));
  cudaMalloc(&d_scaled_heatmap, SCALED_SIZE*SCALED_SIZE*sizeof(int));
  cudaMalloc(&d_blurred_heatmap, SCALED_SIZE*SCALED_SIZE*sizeof(int));
  int size = 0;
  for(int i = 0; i < crowds.size(); i++){
    size += crowds[i]->NumberOfAgents;
  }
  size *= sizeof(int);
  cudaMalloc(&d_DesiredX, size);
  cudaMalloc(&d_DesiredY, size);

  kernel_setup_heat_map(d_heatmap);
 

  cudaMemcpy(blurred_heatmap[0], d_blurred_heatmap,
	     SCALED_SIZE*SCALED_SIZE*sizeof(int),cudaMemcpyDeviceToHost);

  int totalNumAgents = 0;
  for(int i = 0; i < crowds.size(); i++){
    totalNumAgents += crowds[i]->NumberOfAgents;
  }

  tempDesX = new int[totalNumAgents];
  tempDesY = new int[totalNumAgents];
  
  printf("steup done kernel\n");
 
}

int const * const * Ped::Model::getHeatmap() const {
  return blurred_heatmap;
}
int Ped::Model::getHeatmapSize() const {
  return SCALED_SIZE;
}
void *Ped::Model::updateHeatmapPar(void *_notused){

  //Launch kernel
  int totalNumAgents = 0;
  for(int i = 0; i < crowd.size(); i++){
    for(int j = 0; j < crowd[i]->NumberOfAgents; j++){
      tempDesX[totalNumAgents+j] = (int) crowd[i]->DesiredX[j];
      tempDesY[totalNumAgents+j] = (int) crowd[i]->DesiredY[j];
    }
    totalNumAgents += crowd[i]->NumberOfAgents;
  }

  cudaMemcpy(d_DesiredX, tempDesX, totalNumAgents * sizeof(int),
	     cudaMemcpyHostToDevice);
  cudaMemcpy(d_DesiredY, tempDesY, totalNumAgents * sizeof(int),
	     cudaMemcpyHostToDevice);


  kernal_update_heat_map(d_heatmap, d_scaled_heatmap, 
			 d_blurred_heatmap, d_DesiredX,
			 d_DesiredY,
			 totalNumAgents);


  cudaMemcpy(blurred_heatmap[0], d_blurred_heatmap,
	     SCALED_SIZE*SCALED_SIZE*sizeof(int),cudaMemcpyDeviceToHost);
    

}
