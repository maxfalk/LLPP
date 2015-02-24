#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include <map>
#include "ped_crowd.h"
#include "ped_tree.h"

namespace Ped{
  class Ttree;
  class Crowd;
  enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ};
  class Model
  {
  public:
    void setup(std::vector<Crowd*> crowdsInScenario, IMPLEMENTATION mode, 
	       int nrOfThreads);
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
    void omp();
    void seq();
    void pThreads();
    void vector();
    //void cuda();
   

    void doSafeMovment(std::pair<Ped::Crowd*, int> Agent);
    // The maximum quadtree depth
    static const int treeDepth = 10;    

    // Keeps track of the positions of each agent
    Ped::Ttree *tree;

    // Maps the agent to the tree node containing it. Convenience data structure
    // in order to update the tree in case the agent moves.
    std::map<std::pair<Crowd*, int>, Ped::Ttree*> *treehash;

    // Returns the set of neighboring agents for the specified position
    std::set<std::pair<Ped::Crowd*, int> > getNeighbors(int x, int y, int dist) const;
    void getNeighbors(std::list<std::pair<Ped::Crowd*, int> >& neighborList, int x, 
		      int y, int d) const;
  };
}
#endif
