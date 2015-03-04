#ifndef _view_agent_h
#define _view_agent_h

#include <QGraphicsScene>
#include <QGraphicsRectItem>
#include "ped_crowd.h"


class ViewAgent{
 public:
  ViewAgent(float &_X, float &_Y, QGraphicsScene * scene);
  void paint();

 private:
  float &X;  
  float &Y;
  QGraphicsRectItem * rect;

 };





#endif
