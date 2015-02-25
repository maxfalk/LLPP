#ifndef _ped__net__
#define _ped__net__
#include <stdlib.h>

namespace Ped{

  class Net
  {

  public:
    std::pair<Crowd*, int> **net;
    int sizeX;
    int sizeY;
    
    //Functions
    Net(int, int);
    ~Net();
    bool take_pos_atomic(int, int, int);
    void get_neighbors(int x, int y, std::vector<std::pair<int,int> > &neighbors);

  };

}
#endif
