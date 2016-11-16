#include "ros/ros.h"
#include "std_msgs/String.h"
#include "planner/node.h"
#include <vector>       // std::vector
#include <algorithm>    // std::make_heap, std::pop_heap, std::push_heap, std::sort_heap
#include <math.h>  
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GridCells.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

float height;
float width;
float resolution;
float offsetX;
float offsetY;
int received = 0;
int startX = 166;
int startY = 198;
int goalX = 186;
int goalY = 211;

ros::Publisher pub; 


nav_msgs::OccupancyGrid grid;
void mapCallBack(nav_msgs::OccupancyGrid data){
  grid = data;
  resolution = data.info.resolution;
  width = data.info.width;
  height = data.info.height;
  offsetX = data.info.origin.position.x;
  offsetY = data.info.origin.position.y;
  received =1;
}
void readStart(geometry_msgs::PoseWithCovarianceStamped p){
  startX = (p.pose.pose.position.x - offsetX)/resolution -1.5;
  startY = (p.pose.pose.position.y - offsetY)/resolution +0.5;
  received=1;

}
void readGoal(geometry_msgs::PoseStamped p){
  goalX = (p.pose.position.x - offsetX)/resolution -1.5;
  goalY = (p.pose.position.y - offsetY)/resolution +0.5;
  received=1;
}


float dist(float x1, float y1, float x2, float y2){
  float value = sqrt(pow((x2-x1),2) + pow((y2-y1),2));
  return value;
}
struct Comp
{
   bool operator()(const node& node1, const node& node2)
   {
       return node1.cost>=node2.cost;
   }
};
void publishCells(std::vector<node> &nodes){
  //resolution and offset of the map
  int k=0;
  nav_msgs::GridCells cells;
  cells.header.frame_id = "map";
  cells.cell_width = resolution;
  cells.cell_height = resolution;
  //std::cout << "height " <<height << "  width  " << width <<std::endl;
  for(int i = 0;i<nodes.size();i++){

    geometry_msgs::Point point;
    point.x=(nodes[i].nodeX*resolution)+offsetX + (1.5 * resolution);// # added secondary offset 
    point.y=(nodes[i].nodeY*resolution)+offsetY - (.5 * resolution);// # added secondary offset ... Magic ?
    point.z=0;;
    cells.cells.push_back(point);
  }
  pub.publish(cells);
}
  

int count =0;
void Astar(){
  nav_msgs::GridCells cells;
  cells.header.frame_id = "map";
  cells.cell_width = resolution;
  cells.cell_height = resolution;

  std::vector<node> fringe;
  std::vector<node> processed;
  std::make_heap(fringe.begin(),fringe.end(),Comp());
  node firstNode(startX,startY,0,startX,startY,0);
  fringe.push_back(firstNode); std::push_heap (fringe.begin(),fringe.end(),Comp());
  while(true){
  //for(int k =0; k<fringe.size();k++){ std::cout <<fringe[k].cost <<"  ";}std::cout <<std::endl;

    node processingNode = fringe.front();
    //std::cout <<processingNode.nodeY <<std::endl;
    if(processingNode.nodeX ==goalX && processingNode.nodeY == goalY){//if processing goal or fringe empty
      std::cout << "cost is "<< processingNode.realCost <<std::endl;
      break;
    }
    count ++;
    if(fringe.size()==0 || count >100000){
      std::cout << "no path" <<std::endl;
      break;
    }
    std::pop_heap(fringe.begin(),fringe.end(),Comp());
    fringe.pop_back();
    //for(int k =0; k<fringe.size();k++){ std::cout <<fringe[k].nodeY <<"  ";}std::cout <<std::endl;

    bool wasProcessed=false;
    for(unsigned j=0; j<processed.size();j++){
      if(processed[j].nodeX==processingNode.nodeX && processed[j].nodeY==processingNode.nodeY){
        
        wasProcessed=true;
        //std::cout << "was processed" << processed[j].nodeX<<"  " <<processed[j].nodeY <<std::endl;
        break;
      }
    }
    if(wasProcessed){
      continue;
    }
    //std::cout <<"processing " <<processingNode.nodeX <<"," << processingNode.nodeY<< " to find goal  " <<goalX<<"," << goalY << std::endl;

    for(int i =0; i<4; i++){
      int x = processingNode.nodeX;
      int y = processingNode.nodeY;
      switch(i){
        case 0:x++; break;
        case 1:y++; break;
        case 2:x--; break;
        case 3:y--; break;
      }
      if(x <1 || y <1 || x>width || y >height){ 
        std::cout <<"hit bounds" <<std::endl;
        continue;
      }
      if(grid.data[(x+1 + (y-1)*width)]==100){ // if occupied
        //std::cout << "blocked" <<std::endl;
        continue;
      }

      float cost = dist(x,y,goalX,goalY) + processingNode.realCost +1;
      node* newNode = new node(x,y,cost,processingNode.nodeX,processingNode.nodeY,processingNode.realCost +1);
      fringe.push_back(*newNode); std::push_heap (fringe.begin(),fringe.end(),Comp());
    }
    processed.push_back(processingNode);
    publishCells(processed);
    ros::Duration(0.005).sleep(); // sleep for half a second


  }
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "p");

  ros::NodeHandle n;
  pub = n.advertise<nav_msgs::GridCells>("/map_check", 1000);

  ros::Subscriber sub = n.subscribe("/map", 1000, mapCallBack);
  ros::Subscriber goal_sub = n.subscribe("move_base_simple/goal",100,readGoal);
  ros::Subscriber start_sub = n.subscribe("initialpose",100,readStart);

  while(ros::ok()){


  ros::spinOnce();
  if(received){
    //pub.publish((publishCells()));
    std::cout <<"saw msg" <<std::endl;
    Astar();

    received = 0;
  }
  }


  

  return 0;
}