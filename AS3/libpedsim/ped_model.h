#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include <map>
#include "ped_crowd.h"
#include "ped_tree.h"
//#include "ped_agent.h"

#define COL_THREADS 4

namespace Ped{
  class Ttree;

  enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ};
  class Model
  {
  public:
    void setup(std::vector<Crowd*> crowdsInScenario, IMPLEMENTATION mode, 
	       int nrOfThreads, bool parallelCollision);
    void tick();
    const std::vector<Crowd*> getCrowds() const;

    // Updates the treehash, which maps each agent to the current tree node that contains it
    void setResponsibleTree(Ped::Ttree *tree, const int offset);

   
    // Adds an agent to the tree structure
    void placeAgent(const int offset);

    void cleanup();
    //~Model();
  private:
    int nrOfThreads;
    IMPLEMENTATION implementation;
    std::vector<Crowd*> crowds;
    bool parallelCollision;
    void omp();
    void seq();
    void pThreads();
    void vector();
    //void cuda();
    
    std::vector<int> agents;

    void doSafeMovement(int offset);
    // The maximum quadtree depth
    static const int treeDepth = 10;    

    // Keeps track of the positions of each agent
    Ped::Ttree *tree;

    // Maps the agent to the tree node containing it. Convenience data structure
    // in order to update the tree in case the agent moves.
    std::map<const int, Ped::Ttree*> *treehash;

    // Returns the set of neighboring agents for the specified position
    set<const int> getNeighbors(int x, int y, int dist) const;
    void getNeighbors(list<const int>& neighborList, int x, int y, int d) const;
  };
}
#endif
