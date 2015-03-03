#ifndef __ped_net__
#define __ped_net__
#include "ped_crowd.h"
#include <utility>
#include <vector>
#include <stdlib.h>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>


namespace Ped{

  class Net{
  public:
    typedef struct {
      Crowd* first;
      int second;
    }_Npair, *Npair;

    Npair **field;
    int sizeY;
    int sizeX;
    Net() {
      field = NULL;
      sizeY = 0;
      sizeX = 0;
    };
    Net(int x, int y);

    void init(int y,int x);
    ~Net();
    bool take_pos_atomic(int x, int y, Npair newVal);
    void get_neighbors(int x, int y, std::vector<Npair >* neighbors);
  };

}
#endif
