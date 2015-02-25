#include <stdlib.h>
#include <stdio.h>
#include "ped_crowd.h"
#include "ped_net.h"
#include <utility>
using namespace Ped;

Ped::Net::Net(int x, int y){
  printf("he");
  int sizeX = x;
  int sizeY = y;
  net = new std::pair<Crowd*, int>**[x];
  for(int i = 0; i < x; i++){
    net[i] = new std::pair<Crowd*, int>*[y];
  }

  for(int i = 0; i < sizeX; i++){
    for(int j = 0; j < sizeY; i++){
      net[j][j] = NULL;
    }
  }


}

Ped::Net::~Net(){

  for(int i = 0; i < sizeX; i++){
    for(int j = 0; j < sizeY; i++){
      delete net[i][j];
    }
  }

  delete[] *net;
  delete[] net;

}
bool Ped::Net::take_pos_atomic(int x, int y, std::pair<Ped::Crowd*, int> *newVal){

  return __sync_bool_compare_and_swap(&net[x][y],NULL, newVal);

}
void Ped::Net::get_neighbors(int x, int y, std::vector< std::pair<Ped::Crowd*, int>* >* neighbors){

  if(y > 0)
    neighbors->push_back(net[x][y-1]);
  if(x > 0)
    neighbors->push_back(net[x-1][y]);
  if(y > 0 && x > 0)
    neighbors->push_back(net[x-1][y-1]);
  if(y < sizeY)
    neighbors->push_back(net[x][y+1]);
  if(x < sizeX)
    neighbors->push_back(net[x+1][y]);
  if(y < sizeY && x < sizeX)
    neighbors->push_back(net[x+1][y+1]);

}

