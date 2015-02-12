//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//

#include "ParseScenario.h"
#include <string>

/// object constructor
/// \date    2011-01-03
ParseScenario::ParseScenario(QString filename) : QObject(0)
{
  QFile file(filename);
  if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
  {
    return;
  }

  while (!file.atEnd())
  {
    QByteArray line = file.readLine();
    processXmlLine(line);
  }
}

vector<Ped::Crowd*> ParseScenario::getCrowds() const
{
  return crowds;
}

/// Called for each line in the file
void ParseScenario::processXmlLine(QByteArray dataLine)
{
  xmlReader.addData(dataLine);

  while (!xmlReader.atEnd())
  {
    xmlReader.readNext();
    if (xmlReader.isStartElement())
    {
      handleXmlStartElement();
    }
    else if (xmlReader.isEndElement())
    {
      handleXmlEndElement();
    }
  }
}

void ParseScenario::handleXmlStartElement()
{
  if (xmlReader.name() == "waypoint")
  {
    handleWaypoint();	
  }
  else if (xmlReader.name() == "agent")
  {
    handleAgent();
  }
  else if (xmlReader.name() == "addwaypoint")
  {
    handleAddWaypoint();
  }
  else
  {
    // nop, unknown, ignore
  }
}

void ParseScenario::handleXmlEndElement()
{
  if (xmlReader.name() == "agent") {
    //do nothing
  }
}

void ParseScenario::handleWaypoint()
{
  QString id = readString("id");
  float x = readFloat("x");
  float y = readFloat("y");
  float r = readFloat("r");
  
  
  WaypointsX.push_back(x);
  WaypointsY.push_back(y);
  WaypointsR.push_back(r);
  Waypoints[id] = WaypointsX.size()-1;

}
void ParseScenario::handleAgent()
{
  float x = readFloat("x");
  float y = readFloat("y");
  int n = readFloat("n");
  float dx = readFloat("dx");
  float dy = readFloat("dy");
  Ped::Crowd *crowd = new Ped::Crowd(n, WaypointsX.size());
  for (int i = 0; i < n; ++i)
  {
    int xPos = x + qrand()/(RAND_MAX/dx) -dx/2;
    int yPos = y + qrand()/(RAND_MAX/dy) -dy/2;
    crowd->AgentsX[i] = xPos;
    crowd->AgentsY[i] = yPos;
  }
  crowds.push_back(crowd);
}
void ParseScenario::handleAddWaypoint()
{
  QString id = readString("id");
  Ped::Crowd *crowd = crowds.back();
  crowd->WaypointX[Waypoints[id]] = WaypointsX.at(Waypoints[id]);
  crowd->WaypointY[Waypoints[id]] = WaypointsY.at(Waypoints[id]);
  crowd->WaypointR[Waypoints[id]] = WaypointsR.at(Waypoints[id]);

}

float ParseScenario::readFloat(const QString &tag)
{
  return readString(tag).toFloat();
}

QString ParseScenario::readString(const QString &tag)
{
  return xmlReader.attributes().value(tag).toString();
}
