//
// pedsim - A microscopic pedestrian simulation system.
// Copyright (c) 2003 - 2014 by Christian Gloor
//
//
// Adapted for Low Level Parallel Programming 2015
//

#include "ParseScenario.h"
#include <string>
#include <stdio.h>
/// object constructor
/// \date    2011-01-03
ParseScenario::ParseScenario(QString filename, int _vector_mode) : QObject(0)
{
  vector_mode = _vector_mode;
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
  if (xmlReader.name() == "welcome") {
    for(uint i = 0; i < crowds.size(); i++){
      crowds[i]->init();
    }
  }else if (xmlReader.name() == "agent") {
    WayPos = 0;
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
bool ParseScenario::checkForDuplicates(vector<int> TempPositions, int x, int y){
  bool duplicate = false;
  for(uint i=0;i < TempPositions.size();i+=2){
    if(TempPositions[i] == x and 
       TempPositions[i+1] == y){
      duplicate = true;
    }

  }

  return duplicate;
}
void ParseScenario::handleAgent()
{
  float x = readFloat("x");
  float y = readFloat("y");
  int n = readFloat("n");
  float dx = readFloat("dx");
  float dy = readFloat("dy");
  int xPos = 0;
  int yPos = 0;
  vector<int> TempPositions;
 
  for(int i = 0; i < n;i++)
    {
	xPos = x + qrand()/(RAND_MAX/dx) -dx/2;
	yPos = y + qrand()/(RAND_MAX/dy) -dy/2;
	if(checkForDuplicates(TempPositions, xPos, yPos) == false){
	TempPositions.push_back(xPos);
	TempPositions.push_back(yPos);
      }

    }
  
  int realNumAgents = (TempPositions.size())/2;
  printf("NumAgents: %d\n", realNumAgents);
  Ped::Crowd *crowd = new Ped::Crowd(realNumAgents, WaypointsX.size(), vector_mode);  
  for(uint i=0; i < TempPositions.size(); i+=2){
    crowd->AgentsX[i] = TempPositions[i];
    crowd->AgentsY[i] = TempPositions[i+1];
  }

  crowds.push_back(crowd);
}

void ParseScenario::handleAddWaypoint()
{
  QString id = readString("id");
  Ped::Crowd *crowd = crowds.back();
  crowd->WaypointX[WayPos] = WaypointsX.at(Waypoints[id]);
  crowd->WaypointY[WayPos] = WaypointsY.at(Waypoints[id]);
  crowd->WaypointR[WayPos] = WaypointsR.at(Waypoints[id]);
  WayPos++;
}

float ParseScenario::readFloat(const QString &tag)
{
  return readString(tag).toFloat();
}

QString ParseScenario::readString(const QString &tag)
{
  return xmlReader.attributes().value(tag).toString();
}
