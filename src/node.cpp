#include "planner/node.h"

node::node(int x,int y,float totalCost, int x2, int y2,int trueCost){
	nodeX=x;
	nodeY=y;
	cost = totalCost;
	parentX = x2;
	parentY = y2;
	realCost =trueCost;
}