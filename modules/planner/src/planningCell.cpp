#include "planner/planningCell.h"

PlanningCell::PlanningCell()
{
   _skillClass = SC_NONE;
   _skillCost = 1.0;
   _occupancy = 0.5;
   _vegetation = 0;
   _utility = 0.0;
   _obstacleEdge = NULL;
   _x = -1;
   _z = -1;
}
   
PlanningCell::~PlanningCell()
{
}

void PlanningCell::setCoords(int x, int z)
{
   _x = x;
   _z = z;
}

void PlanningCell::setSkillClass(enum SKILL_CLASSES cls)
{
   _skillClass = cls;
}

void PlanningCell::setSkillCost(double cost)
{
   _skillCost = cost;
}

double PlanningCell::getSkillCost() const
{
   return _skillCost;
}

void PlanningCell::setObstacleEdge(ObstacleEdge * oe)
{
   _obstacleEdge = oe;
}
   
ObstacleEdge* PlanningCell::getObstacleEdge() const
{
   return _obstacleEdge;
}

double PlanningCell::getDistance() const
{
   return _distance;
}

void PlanningCell::setDistance(const double dist)
{
   _distance = dist;
}


