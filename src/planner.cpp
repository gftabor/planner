#include "ros/ros.h"
#include <tf/transform_listener.h>
#include "std_msgs/String.h"
#include "planner/node.h"
#include <vector>       // std::vector
#include <algorithm>    // std::make_heap, std::pop_heap, std::push_heap, std::sort_heap
#include <math.h>  
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GridCells.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include <iostream>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

float height;
float width;
float resolution;
float offsetX;
float offsetY;
int receivedMap = 0;
int receivedGoal = 0;
int beginAStar = 0;
int startX = 166;
int startY = 198;
int goalX = 186;
int goalY = 211;

ros::Publisher pub;
ros::Publisher pubWay;



nav_msgs::OccupancyGrid grid;
void mapCallBack(nav_msgs::OccupancyGrid data){
  std::cout << "saw map" <<std::endl;
  grid = data;
  resolution = data.info.resolution;
  width = data.info.width;
  height = data.info.height;
  offsetX = data.info.origin.position.x;
  offsetY = data.info.origin.position.y;
  receivedMap = 1;
  if(receivedGoal)
    beginAStar=1;
}
void readStart(){
  tf::StampedTransform transform;
  tf::TransformListener listener;
  listener.waitForTransform("odom", "base_footprint", ros::Time(0), ros::Duration(10.0));
  listener.lookupTransform("odom", "base_footprint", ros::Time(0), transform);
  startX = (transform.getOrigin().x() - offsetX)/resolution -1.5;
  startY = (transform.getOrigin().y() - offsetY)/resolution +0.5;
}
void readGoal(geometry_msgs::PoseStamped p){
  goalX = (p.pose.position.x - offsetX)/resolution -1.5;
  goalY = (p.pose.position.y - offsetY)/resolution +0.5;
  receivedGoal = 1;
    if(receivedMap)
      beginAStar=1;
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
void publishWay(nav_msgs::Path path){
  std::cout << "size real  " << path.poses.size() << std::endl;
  for(int i=0; i <path.poses.size(); i++){
    std::cout << path.poses[i].pose.position.x<<" "<< path.poses[i].pose.position.y <<std::endl;

  }
  pubWay.publish(path);
}
void createWay(std::vector<node> &processed){
  nav_msgs::Path my_path_bitch;

  int currentX = goalX;
  int currentY = goalY;
  my_path_bitch.header.frame_id="map";
  while((currentX != startX || currentY != startY)){
    geometry_msgs::PoseStamped pose; 
    pose.header.frame_id="map";
    pose.pose.position.x = (currentX*resolution)+offsetX + (1.5 * resolution);
    pose.pose.position.y = (currentY*resolution)+offsetY - (.5 * resolution);
    //you fixed the pose after you added it to the list. 
    pose.pose.orientation.w =1;
    my_path_bitch.poses.push_back(pose);
    for(long i=0; i<processed.size(); i++){
      if(processed[i].nodeX == currentX && processed[i].nodeY == currentY){
        currentX = processed[i].parentX;
        currentY = processed[i].parentY;
        break;
      }
    }   
  }
  pubWay.publish(my_path_bitch);

}
  

int count =0;
void Astar(){  
  std::vector<node> fringe;
  std::vector<node> processed;
  std::make_heap(fringe.begin(),fringe.end(),Comp());
  node firstNode(startX,startY,0,startX,startY,0);
  fringe.push_back(firstNode); std::push_heap (fringe.begin(),fringe.end(),Comp());
  while(true){//process nodes until you process goal
    node processingNode = fringe.front();


    //if any end conditions are done stop immediately
    if(processingNode.nodeX ==goalX && processingNode.nodeY == goalY){//if processing goal or fringe empty
        std::cout << "cost is "<< processingNode.realCost <<std::endl;
        processed.push_back(processingNode);

        break;
      }
    count ++;
    if(fringe.size()==0 || count >500000){
      std::cout << "no path" <<std::endl;
      break;
    }


    //remove the point from list
    std::pop_heap(fringe.begin(),fringe.end(),Comp());
    fringe.pop_back();

    //ensure point has not been processed
    bool wasProcessed=false;
    for(unsigned j=0; j<processed.size();j++){
      if(processed[j].nodeX==processingNode.nodeX && processed[j].nodeY==processingNode.nodeY){
        wasProcessed=true;
        break;
      }
    }
    if(wasProcessed){
      continue;
    }


    //process by creating new points that are neighbours to currently being processed node
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
        continue;
      }

      float cost = dist(x,y,goalX,goalY) + processingNode.realCost +1;
      node* newNode = new node(x,y,cost,processingNode.nodeX,processingNode.nodeY,processingNode.realCost +1);
      fringe.push_back(*newNode); std::push_heap (fringe.begin(),fringe.end(),Comp());
    }

    //print current gridcells of processed nodes
    processed.push_back(processingNode);
    publishCells(processed);
    //ros::Duration(0.0005).sleep(); // sleep for half a second
  }  
  //create and publish waypoints
  createWay(processed);
  
}

int main(int argc, char **argv)
{
 
  ros::init(argc, argv, "p");

  ros::NodeHandle n;
  pub = n.advertise<nav_msgs::GridCells>("/map_check", 1000);
  pubWay = n.advertise<nav_msgs::Path>("/totes_path", 1000);

  ros::Subscriber sub = n.subscribe("/map_real", 1000, mapCallBack);
  ros::Subscriber goal_sub = n.subscribe("move_base_simple/goal",100,readGoal);

  while(ros::ok()){


  ros::spinOnce();
  if(beginAStar==1){
    //pub.publish((publishCells()));
    std::cout <<"saw msg" <<std::endl;
    readStart();
    beginAStar = 0;
    Astar();
  }
  }


  

  return 0;
}
