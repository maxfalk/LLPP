//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//

#ifndef _ped_tree_h_
#define _ped_tree_h_ 1

#ifdef WIN32
#define LIBEXPORT __declspec(dllexport)
#else
#define LIBEXPORT
#endif

#include <set>
#include <list>
#include <utility> 
#include "ped_model.h"
#include "ped_crowd.h"

namespace Ped {
  class Model;
  class Crowd;

  class LIBEXPORT Ttree {
    friend class Ped::Model;
    
  public:
    Ttree(Ped::Ttree *root, std::map<std::pair<Ped::Crowd*, int>, Ped::Ttree*> *treehash, 
	  int depth, int maxDepth, double x, double y, double w, double h);
    virtual ~Ttree();
    
    virtual void clear();
    
    virtual void addAgent(std::pair<Ped::Crowd*, int> Agent);
    virtual void moveAgent(std::pair<Ped::Crowd*,  int> Agent);
    virtual bool removeAgent(std::pair<Ped::Crowd*, int> Agent);
    
    virtual std::set<std::pair<Ped::Crowd*, int> > getAgents() const;
    virtual void getAgents(std::list<std::pair<Ped::Crowd*, int> >& outputList) const;
    
    virtual bool intersects(double px, double py, double pr) const;
    
    double getx() const { return x; };
    double gety() const { return y; };
    double getw() const { return w; };
    double geth() const { return h; };

    int getdepth() const { return depth; };
  protected:
    virtual void addChildren();
    Ttree* getChildByPosition(double x, double y);
    int cut();
    pthread_mutex_t mutexsum;
    bool isleaf;
    double x;
    double y;
    double w;
    double h;
    int depth;
    int maxDepth;
    
    Ttree *tree1;
    Ttree *tree2;
    Ttree *tree3;
    Ttree *tree4;
    Ttree *root;
  private:
    std::map<std::pair<Ped::Crowd*, int>, Ped::Ttree*> *treehash;
    std::set<std::pair<Ped::Crowd*, int> > agents;
  };
}

#endif
