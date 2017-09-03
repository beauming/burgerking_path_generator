#ifndef __CPathGenerator_H_
#define __CPathGenerator_H_

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "tf/transform_listener.h"
#include "std_msgs/Header.h"
#include "burgerking_path_generator/path.h"

#include <fstream>
#include <string>
#include <math.h>


#define pi 3.141592

enum occupancyValue{
    unknown = 0,
    empty,
    barrier
};


enum connection
{
    up = 1,
    right,
    down,
    left,
    start,
    goal
};






struct Node
{
    int H; 
    connection from;
    
    std::vector<std::vector<std::pair<int, int> > > nodeComponent;
      


};

class CPathGenerator
{
public: 
    CPathGenerator();
     ~CPathGenerator();

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg);

    void mkOccupancyGridMap(int rows, int cols);

    void getRobotState();

    void searchArea();

    void sendPathmsg();

    bool pathFollowing(float x, float y);

    void pathCheck();

    void searchGoal();

    Node StartNode;
    Node GoalNode;

    Node ParentNode;
    Node SelectNode;

    std::vector<Node> ChildNode;
    std::vector<Node> OpenNode;
    std::vector<Node> ClosedNode;

    std::vector<std::pair<int, int> > goal_Candidate;


    void pathGenerate();

    int H_function(Node nd);

    bool NodeCapabilityCheck(Node nd);

    bool NewNodeCheck(Node nd);

    Node ChooseSelectNode(Node nd1, Node nd2);

    void SetPathNode();
    
    std::vector<Node> PathNode;

    

    nav_msgs::MapMetaData info;

    std::vector<std::vector<int> > gridmap;

    tf::TransformListener listener;   
    tf::StampedTransform transform;

    float robotPos_x;
    float robotPos_y;
    float robot_angle;

    int robotGrid_x;
    int robotGrid_y;

    int old_robotGrid_x;
    int old_robotGrid_y;


    std::vector<std::vector<std::pair<float,float> > > searchingMatrix;
    std::vector<std::vector<std::pair<int, int> > > cell_searchingMatrix;
    std::vector<std::vector<int> > value_searchingMatrix;

    std::vector<std::vector<std::pair<int, int> > > path_Matrix;
    int path_Matrix_rows;
    int path_Matrix_cols;
 //   std::vector<std::vector<int> > check_Matrix;



    float size_r;
    int searchVertical;
    int half_searchVertical = searchVertical/2;
    
    int searchDirectional;
    int sDirNum;


    ros::NodeHandle handle;
    ros::Publisher path_pub;

    float linearEq_a;
    float linearEq_b;
    float linearEq_c;

    int Nd_rows;
    int Nd_cols;

    int test;

};



#endif //__CPathGenerator_H_