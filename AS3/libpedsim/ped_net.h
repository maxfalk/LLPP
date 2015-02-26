#ifndef __ped_net__
#define __ped_net__
#include "ped_crowd.h"
#include <stdlib.h>
#include <utility>
#include <vector>
namespace Ped{

  class Net{
  public:
    std::pair<Crowd*, int> ***net;
    int sizeY;
    int sizeX;
    Net(int x, int y);
    ~Net();
    bool take_pos_atomic(int x, int y, std::pair<Ped::Crowd*, int> *newVal);
    void get_neighbors(int x, int y, std::vector<std::pair<Ped::Crowd*, int>* >* neighbors);

  };


}

#endif
