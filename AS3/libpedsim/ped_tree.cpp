//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
#include "ped_tree.h"

#include <cassert>
#include <cstddef>
#include <algorithm>

using namespace std;


/// Description: set intial values
/// \author  chgloor
/// \date    2012-01-28
Ped::Ttree::Ttree(Ped::Ttree *root,std::map<const int, Ped::Ttree*> *treehash, int pdepth, int pmaxDepth, double px, double py, double pw, double ph) {
    // more initializations here. not really necessary to put them into the initializator list, too.
  this->root = root != NULL ? root: this;
  this->treehash = treehash;
    isleaf = true;
    x = px;
    y = py;
    w = pw;
    h = ph;
    depth = pdepth;
    maxDepth = pmaxDepth;
    tree1 = NULL;
    tree2 = NULL;
    tree3 = NULL;
    tree4 = NULL;
};


/// Destructor. Deleted this node and all its children. If there are any agents left, they are removed first (not deleted).
/// \author  chgloor
/// \date    2012-01-28
Ped::Ttree::~Ttree() {
    clear();
}


void Ped::Ttree::clear() {
    if(isleaf) {
        agents.clear();
    }
    else {
        tree1->clear();
        delete tree1;
        tree2->clear();
        delete tree2;
        tree3->clear();
        delete tree3;
        tree4->clear();
        delete tree4;
        isleaf = true;
    }
}

bool cmp(const int offsetA, const int offsetB) {
  return AgentsX[offsetA] == AgentsX[offsetB] && AgentsY[offsetA] == AgentsY[offsetB];
}

/// Adds an agent to the tree. Searches the right node and adds the agent there.
/// If there are too many agents at that node allready, a new child is created.
/// \author  chgloor
/// \date    2012-01-28
/// \param   *a The agent to add
void Ped::Ttree::addAgent(const int offset) {
    if (isleaf) {
        agents.insert(offset);
	//model->setResponsibleTree(this, a);
	(*treehash)[offset] = this;
    }
    else {
        if ((AgentsX[offset] >= x+w/2) && (AgentsY[offset] >= y+h/2)) tree3->addAgent(offset); // 3
        if ((AgentsX[offset] <= x+w/2) && (AgentsY[offset] <= y+h/2)) tree1->addAgent(offset); // 1
        if ((AgentsX[offset] >= x+w/2) && (AgentsY[offset] <= y+h/2)) tree2->addAgent(offset); // 2
        if ((AgentsX[offset] <= x+w/2) && (AgentsY[offset] >= y+h/2)) tree4->addAgent(offset); // 4
    }

    if (agents.size() > 8 && depth < maxDepth) {
        isleaf = false;
        addChildren();
        while (!agents.empty()) {
            const int offset = (*agents.begin());
            if ((AgentsX[offset] >= x+w/2) && (AgentsY[offset] >= y+h/2)) tree3->addAgent(offset); // 3
            if ((AgentsX[offset] <= x+w/2) && (AgentsY[offset] <= y+h/2)) tree1->addAgent(offset); // 1
            if ((AgentsX[offset] >= x+w/2) && (AgentsY[offset] <= y+h/2)) tree2->addAgent(offset); // 2
            if ((AgentsX[offset] <= x+w/2) && (AgentsY[offset] >= y+h/2)) tree4->addAgent(offset); // 4
            agents.erase(offset);
        }
    }
}


/// A little helper that adds child nodes to this node
/// \author  chgloor
/// \date    2012-01-28
void Ped::Ttree::addChildren() {
  tree1 = new Ped::Ttree(root,treehash, depth+1, maxDepth,  x, y, w/2, h/2);
  tree2 = new Ped::Ttree(root,treehash, depth+1, maxDepth,  x+w/2, y, w/2, h/2);
  tree3 = new Ped::Ttree(root,treehash, depth+1, maxDepth,  x+w/2, y+h/2, w/2, h/2);
  tree4 = new Ped::Ttree(root,treehash, depth+1, maxDepth,  x, y+h/2, w/2, h/2);
}


Ped::Ttree* Ped::Ttree::getChildByPosition(double xIn, double yIn) {
    if((xIn <= x+w/2) && (yIn <= y+h/2))
        return tree1;
    if((xIn >= x+w/2) && (yIn <= y+h/2))
        return tree2;
    if((xIn >= x+w/2) && (yIn >= y+h/2))
        return tree3;
    if((xIn <= x+w/2) && (yIn >= y+h/2))
        return tree4;

    // this should never happen
    return NULL;
}


/// Updates the tree structure if an agent moves. Removes the agent and places it again, if outside boundary.
/// If an this happens, this is O(log n), but O(1) otherwise.
/// \author  chgloor
/// \date    2012-01-28
/// \param   *a the agent to update
void Ped::Ttree::moveAgent(const int offset) {
    if ((AgentsX[offset] < x) || (AgentsX[offset] > (x+w)) || (AgentsY[offset] < y) || (AgentsY[offset] > (y+h))) {
        agents.erase(offset);
        root->addAgent(offset);
    }
}


bool Ped::Ttree::removeAgent(const int offset) {
    if(isleaf) {
        size_t removedCount = agents.erase(offset);
        return (removedCount > 0);
    }
    else {
        return getChildByPosition(AgentsX[offset], AgentsY[offset])->removeAgent(offset);
    }
}


/// Checks if this tree node has not enough agents in it to justify more child nodes. It does this by checking all
/// child nodes, too, recursively. If there are not enough children, it moves all the agents into this node, and deletes the child nodes.
/// \author  chgloor
/// \date    2012-01-28
/// \return  the number of agents in this and all child nodes.
int Ped::Ttree::cut() {
    if (isleaf)
        return agents.size();

    int count = 0;
    count += tree1->cut();
    count += tree2->cut();
    count += tree3->cut();
    count += tree4->cut();
    if (count < 5) {
        assert(tree1->isleaf == true);
        assert(tree2->isleaf == true);
        assert(tree3->isleaf == true);
        assert(tree4->isleaf == true);
        agents.insert(tree1->agents.begin(), tree1->agents.end());
        agents.insert(tree2->agents.begin(), tree2->agents.end());
        agents.insert(tree3->agents.begin(), tree3->agents.end());
        agents.insert(tree4->agents.begin(), tree4->agents.end());
        isleaf = true;
        for (set<const int>::iterator it = agents.begin(); it != agents.end(); ++it) {
            const int offset = (*it);
	    (*treehash)[offset] = this;
        }
        delete tree1;
        delete tree2;
        delete tree3;
        delete tree4;
    }
    return count;
}


/// Returns the set of agents that is stored within this tree node
/// \author  chgloor
/// \date    2012-01-28
/// \return  The set of agents
/// \todo This might be not very efficient, since all childs are checked, too. And then the set (set of pointer, but still) is being copied around.
set<const int> Ped::Ttree::getAgents() const {
    if (isleaf)
        return agents;

    set<const int> ta;
    set<const int> t1 = tree1->getAgents();
    set<const int> t2 = tree2->getAgents();
    set<const int> t3 = tree3->getAgents();
    set<const int> t4 = tree4->getAgents();
    ta.insert(t1.begin(), t1.end());
    ta.insert(t2.begin(), t2.end());
    ta.insert(t3.begin(), t3.end());
    ta.insert(t4.begin(), t4.end());
    return ta;
}

void Ped::Ttree::getAgents(list<const int>& outputList) const {
    if(isleaf) {
      for (set<const int>::iterator it = agents.begin(); it != agents.end(); ++it) {
  	    const int currentAgent = (*it);
            outputList.push_back(currentAgent);
      }
    }
    else {
        tree1->getAgents(outputList);
        tree2->getAgents(outputList);
        tree3->getAgents(outputList);
        tree4->getAgents(outputList);
    }
}


/// Checks if a point x/y is within the space handled by the tree node, or within a given radius r
/// \author  chgloor
/// \date    2012-01-29
/// \return  true if the point is within the space
/// \param   px The x co-ordinate of the point
/// \param   py The y co-ordinate of the point
/// \param   pr The radius
bool Ped::Ttree::intersects(double px, double py, double pr) const {
    if (((px+pr) >= x) && ((px-pr) <= (x+w)) && ((py+pr) >= y) && ((py-pr) <= (y+h)))
        return true; // x+-r/y+-r is inside
    else
        return false;
}