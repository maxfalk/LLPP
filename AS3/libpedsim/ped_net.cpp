#include <stdlib.h>
#include <stdio.h>
#include "ped_crowd.h"
#include "ped_net.h"
#include <utility>
using namespace Ped;

Ped::Net::Net(int x, int y){

  field = NULL;
  sizeY = 0;
  sizeX = 0;
  init(x, y);

}
Ped::Net::~Net(){

  for(int i = 0; i < sizeX; i++){
    for(int j = 0; j < sizeY; i++){
      delete field[i][j];
    }
  }

  delete[] *field;
  delete[] field;

}
void Ped::Net::init(int x, int y){

  sizeX = x;
  sizeY = y;
  field = (Npair **)malloc(sizeof(Npair*)*sizeX);

  
  for(int i = 0; i < sizeX; i++){
    field[i] = (Npair*) malloc(sizeof(Npair) * sizeY);
  }

  for(int i = 0; i < sizeX; i++){
    for(int j = 0; j < sizeY; j++){
      field[i][j] = NULL;
    }
  }

}

bool Ped::Net::take_pos_atomic(int x, int y, Npair newVal){

  return __sync_bool_compare_and_swap(&field[x][y], NULL, newVal);
}
void Ped::Net::get_neighbors(int x, int y, std::vector<Npair>* neighbors){

  if(y > 0 && field[x][y-1] != NULL)
    neighbors->push_back(field[x][y-1]);
  if(x > 0 && field[x-1][y] != NULL)
    neighbors->push_back(field[x-1][y]);
  if(y > 0 && x > 0 && field[x-1][y-1] != NULL)
    neighbors->push_back(field[x-1][y-1]);
  if(y < sizeY && field[x][y+1] != NULL)
    neighbors->push_back(field[x][y+1]);
  if(x < sizeX && field[x+1][y] != NULL)
    neighbors->push_back(field[x+1][y]);
  if(y < sizeY && x < sizeX && field[x+1][y+1] != NULL)
    neighbors->push_back(field[x+1][y+1]);

}

