#include "ViewAgent.h"

#include "MainWindow.h"

#include <QGraphicsItemAnimation>

ViewAgent::ViewAgent(float &_X, float &_Y,QGraphicsScene * scene) 
  :X(_X), Y(_Y) 
{

  QBrush blueBrush(Qt::green);
  QPen outlinePen(Qt::black);
  outlinePen.setWidth(2);
 
  rect =  scene->addRect(MainWindow::cellToPixel(X),
			 MainWindow::cellToPixel(Y),
			 MainWindow::cellsizePixel-1 ,
			 MainWindow::cellsizePixel-1 , 
			 outlinePen, blueBrush);

}


void ViewAgent::paint(){
  rect->setRect(MainWindow::cellToPixel(X),
		MainWindow::cellToPixel(Y),
		MainWindow::cellsizePixel-1,
		MainWindow::cellsizePixel-1);

  //Todo: animate movement
}
