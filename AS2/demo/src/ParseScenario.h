//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//

#ifndef _parsescenario_h_
#define _parsescenario_h_

#include "ped_crowd.h"
#include <QtCore>
#include <QXmlStreamReader>
#include <vector>

using namespace std;

class ParseScenario : public QObject
{
  Q_OBJECT

public:
  ParseScenario(QString file);
  vector<Ped::Crowd*> getCrowds() const; 

  private slots:
  void processXmlLine(QByteArray data);
	
private:
  QXmlStreamReader xmlReader;

  //Crowd
  vector<Ped::Crowd*> crowds;

  //Global
  map<QString, int> Waypoints;
  vector<float> WaypointsX;
  vector<float> WaypointsY;
  vector<float> WaypointsR;



  void handleWaypoint();
  void handleAgent();
  void handleAddWaypoint();
  void handleXmlStartElement();
  void handleXmlEndElement();

  QString readString(const QString &tag);
  float readFloat(const QString &tag);
};

#endif
