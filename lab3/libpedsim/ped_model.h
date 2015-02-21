#ifndef _ped_model_h_
#define _ped_model_h_
#include <vector>
#include <map>

#include "ped_tree.h"
#include "ped_agent.h"


namespace Ped{
  class Tagent;
  class Ttree;

  enum IMPLEMENTATION {CUDA, VECTOR, OMP, PTHREAD, SEQ};
  class Model
  {
  public:
    void setup(std::vector<Tagent*> agentsInScenario);
    void tick();
    const std::vector<Tagent*> getAgents() const;

    ////////////
    /// THIS IS NEW
    ///////////////////////////////////////////////

    // Updates the treehash, which maps each agent to the current tree node that contains it
    void setResponsibleTree(Ped::Ttree *tree, const Ped::Tagent *agent);

   
    // Adds an agent to the tree structure
    void placeAgent(const Ped::Tagent *a);

    // Cleans up the tree and restructures it. Worth calling every now and then.
    void cleanup();
    ~Model();
    ////////////
    /// END NEW
    ///////////////////////////////////////////////
    
  private:
    IMPLEMENTATION implementation;   
    std::vector<Tagent*> agents;

    ////////////
    /// THIS IS NEW
    ///////////////////////////////////////////////
    void doSafeMovement( Ped::Tagent *agent);
    // The maximum quadtree depth
    static const int treeDepth = 10;    

    // Keeps track of the positions of each agent
    Ped::Ttree *tree;

    // Maps the agent to the tree node containing it. Convenience data structure
    // in order to update the tree in case the agent moves.
    std::map<const Ped::Tagent*, Ped::Ttree*> *treehash;

    // Returns the set of neighboring agents for the specified position
    set<const Ped::Tagent*> getNeighbors(int x, int y, int dist) const;
    void getNeighbors(list<const Ped::Tagent*>& neighborList, int x, int y, int d) const;

    ////////////
    /// END NEW
    ///////////////////////////////////////////////

  };
}
#endif
