#ifndef cluster_h
#define cluster_h

#include <algorithm>

class node
{
  public: 
    node(int x,int y,float totalCost, int x2, int y2,int trueCost);
    int nodeX;
    int nodeY;
    float cost;
    int parentX;
    int parentY;
    int realCost;


  private:
    
};

#endif