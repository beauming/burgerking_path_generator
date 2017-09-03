#include "CPathGenerator.h"


CPathGenerator::CPathGenerator():size_r(0.05),searchVertical(5),path_Matrix_rows(0),path_Matrix_cols(0),
old_robotGrid_x(0),old_robotGrid_y(0),linearEq_a(0),linearEq_b(0),linearEq_c(0)
{

}

CPathGenerator::~CPathGenerator()
{
    gridmap.clear();
    searchingMatrix.clear();
    cell_searchingMatrix.clear();
    value_searchingMatrix.clear();
    path_Matrix.clear();

}


void CPathGenerator::mkOccupancyGridMap(int rows, int cols)
{

}



void CPathGenerator::getRobotState()
{
    listener.waitForTransform("/map","/base_link",ros::Time(0),ros::Duration(10.0));
    listener.lookupTransform("/map","/base_link",ros::Time(0),transform);

   // [m] 
    robotPos_x = transform.getOrigin().x();
    robotPos_y = transform.getOrigin().y();
    ROS_INFO("x = %f, y = %f",robotPos_x, robotPos_y);

    // robot orientation
    robot_angle = tf::getYaw(transform.getRotation());
    ROS_INFO("angle = %f",robot_angle);

    // [pixel]
    robotGrid_x = floor(0.5+(robotPos_x-info.origin.position.x)/info.resolution);
    robotGrid_y = floor(0.5+(robotPos_y-info.origin.position.y)/info.resolution);

    // ROS_INFO("info.resolution = %f\n",info.resolution);
    // ROS_INFO("info.origin.position.x = %f\n",info.origin.position.x);
    ROS_INFO("cell_x = %d, cell_y = %d",robotGrid_x,robotGrid_y);


    if(!pathFollowing(robotPos_x,robotPos_y)){

        path_pub = handle.advertise<burgerking_path_generator::path>("path_msg",10);
        burgerking_path_generator::path pathmsg;   

        pathmsg.distance = 0.0;
        path_pub.publish(pathmsg);
        ROS_INFO("robot don't path following. send path distance = 0.0\n");

    }


    if( robotGrid_x == old_robotGrid_x && robotGrid_y == old_robotGrid_y)
    {
        searchArea();
        sendPathmsg();
    }


    old_robotGrid_x = robotGrid_x;
    old_robotGrid_y = robotGrid_y;

}



bool CPathGenerator::pathFollowing(float x, float y)
{
    if( linearEq_a == 0 && linearEq_b == 0 && linearEq_c == 0){
        return true;
    } 
    // point to line distance 
    float d = abs(linearEq_a*x + linearEq_b*y + linearEq_c)/sqrt(linearEq_a*linearEq_a + linearEq_b*linearEq_b);

    if( d < 0.05){
        return true;
    }else{
        return false;
    }

}





void CPathGenerator::searchArea()
{
    // size_r : length from robot(sensor)  [m]
    // searchVertical : odd number only ( center is robot  )
    // searchDirectional : searching line number to robot direction ( distance between two line is equal to size_r )  
    bool searchflag = true;
 
    float centerPos_x = robotPos_x;
    float centerPos_y = robotPos_y;    
    
    sDirNum = 5;
    searchDirectional = sDirNum;
    

    while(searchflag)
    {
        // row size 
        searchingMatrix.resize(searchDirectional);
        cell_searchingMatrix.resize(searchDirectional);
        value_searchingMatrix.resize(searchDirectional);

        // col size
        for(unsigned int i=searchDirectional-sDirNum; i < searchDirectional; i++)
        {
            searchingMatrix[i].resize(searchVertical);
            cell_searchingMatrix[i].resize(searchVertical);
            value_searchingMatrix[i].resize(searchVertical);
        }    


        // searching area position
        for(int a = searchDirectional-sDirNum; a < searchDirectional; a++)
        {
            if(a == searchDirectional-sDirNum){
                searchingMatrix[searchDirectional-sDirNum][0].first = centerPos_x + (half_searchVertical*size_r)*cosf(robot_angle-(pi/2));
                searchingMatrix[searchDirectional-sDirNum][0].second = centerPos_y + (half_searchVertical*size_r)*sinf(robot_angle-(pi/2));
            }else{

                searchingMatrix[a][0].first = searchingMatrix[a-1][0].first + size_r*cosf(robot_angle);
                searchingMatrix[a][0].second = searchingMatrix[a-1][0].second + size_r*sinf(robot_angle);
            }
            
                for(int b=1; b < searchVertical; b++)
                {
                    searchingMatrix[a][b].first = searchingMatrix[a][b-1].first + size_r*cosf(robot_angle+(pi/2));
                    searchingMatrix[a][b].second = searchingMatrix[a][b-1].second + size_r*sinf(robot_angle+(pi/2));
                }
        }

    /*
        ROS_INFO("searchingMatrix\n");
        for(int i=searchDirectional-sDirNum; i<searchDirectional; i++){
            for(int j=0; j<searchVertical; j++){
                printf("(%f,%f)",searchingMatrix[i][j].first, searchingMatrix[i][j].second);
            }
            printf("\n");
        }
    */
        // 1. x resolution & floor()
        for(int i=searchDirectional-sDirNum; i<searchDirectional; i++){
            for(int j=0; j<searchVertical; j++){
                cell_searchingMatrix[i][j].first = floor((searchingMatrix[i][j].first/info.resolution)+0.5);
                cell_searchingMatrix[i][j].second = floor((searchingMatrix[i][j].second/info.resolution)+0.5);
            }
        }

    /*
        ROS_INFO("1. cell_searchingMatrix\n");
        for(int i=searchDirectional-sDirNum; i<searchDirectional; i++){
            for(int j=0; j<searchVertical; j++){
                printf("(%d,%d)",cell_searchingMatrix[i][j].first, cell_searchingMatrix[i][j].second);
            }
            printf("\n");
        }
    */

        // 2. transformation [x, y] =>[-y,x]
        for(int i=searchDirectional-sDirNum; i<searchDirectional; i++){
            for(int j=0; j<searchVertical; j++){
                int tmp = cell_searchingMatrix[i][j].first;
                cell_searchingMatrix[i][j].first = cell_searchingMatrix[i][j].second*-1;
                cell_searchingMatrix[i][j].second = tmp;
            }
        }
        
    /*    
        ROS_INFO("2. cell_searchingMatrix\n");
        for(int i=searchDirectional-sDirNum; i<searchDirectional; i++){
            for(int j=0; j<searchVertical; j++){
                printf("(%d,%d)",cell_searchingMatrix[i][j].first, cell_searchingMatrix[i][j].second);
            }
            printf("\n");
        }
    */

        // 3. movement 
        int moveValue = floor(0.5+(0-info.origin.position.x)/info.resolution);
        for(int i=searchDirectional-sDirNum; i<searchDirectional; i++){
            for(int j=0; j<searchVertical; j++){
                cell_searchingMatrix[i][j].first += moveValue;
                cell_searchingMatrix[i][j].second += moveValue;
            }
        }

    /*
        ROS_INFO("3. cell_searchingMatrix\n");
        for(int i=0; i<searchDirectional; i++){
            for(int j=0; j<searchVertical; j++){
                printf("(%d,%d)",cell_searchingMatrix[i][j].first, cell_searchingMatrix[i][j].second);
            }
            printf("\n");
        }
    */

        ROS_INFO("value_searchingMatrix\n");
        // searching area occupancy value    
        for(int i=searchDirectional-sDirNum; i<searchDirectional; i++){
            for(int j=0; j<searchVertical; j++){
                value_searchingMatrix[i][j] = gridmap[cell_searchingMatrix[i][j].first][cell_searchingMatrix[i][j].second];
                printf("%d ",value_searchingMatrix[i][j]);
            }
            printf("\n");
        }


        // occupancy value check
        bool escape = false;

        for(int i=searchDirectional-sDirNum; i<searchDirectional; i++)
        {
            for(int j=0; j<searchVertical; j++)
            {
                if( value_searchingMatrix[i][j] == barrier)
                {
                    path_Matrix_rows = searchDirectional-sDirNum;
                    path_Matrix_cols = searchVertical;
                    path_Matrix.resize(path_Matrix_rows);                    

                    for(int a=0; a<path_Matrix_rows; a++){
                        path_Matrix[a].resize(searchVertical);
                    }

                    for(int m=0; m<path_Matrix_rows; m++){
                        for(int n=0; n<path_Matrix_cols; n++){
                            path_Matrix[m][n].first = cell_searchingMatrix[m][n].first;
                            path_Matrix[m][n].second = cell_searchingMatrix[m][n].second;
                        }                        
                    }

                    searchflag = false;
                    escape = true;
                    ROS_INFO("barrier , searchDirectional = %d\n",searchDirectional);
                    break;                    
                }
            }

            if(escape == true)
                break;

            if( i == searchDirectional - 1){
                centerPos_x = searchingMatrix[searchDirectional-1][half_searchVertical].first;
                centerPos_y = searchingMatrix[searchDirectional-1][half_searchVertical].second;
                searchDirectional += sDirNum;
                break;
            }
        }
        
    }


}


void CPathGenerator::sendPathmsg()
{
    path_pub = handle.advertise<burgerking_path_generator::path>("path_msg",10);
    burgerking_path_generator::path pathmsg;   


    if(searchDirectional > sDirNum)
    {
        float toX = searchingMatrix[searchDirectional-sDirNum][half_searchVertical].first;
        float toY = searchingMatrix[searchDirectional-sDirNum][half_searchVertical].second;

        float path = sqrt((toX-robotPos_x)*(toX-robotPos_x)+(toY-robotPos_y)*(toY-robotPos_y));

        pathmsg.distance = path;
        path_pub.publish(pathmsg);
        ROS_INFO("send path distance = %f\n",path);


        // linear path equation //
        float x1 = robotPos_x;
        float y1 = robotPos_y;
        float x2 = toX;
        float y2 = toY;

        if( x1 == x2 ){
            linearEq_a = 1;
            linearEq_b = 0;
            linearEq_c = x1*-1;
        }
        else if( y1 == y2 ){
            linearEq_a = 0;
            linearEq_b = 1;
            linearEq_c = y1*-1;
        }else{

            linearEq_a = (y1-y2)/(x1-x2);
            linearEq_b = -1;
            linearEq_c = y1 - (x1*linearEq_a);
        }
    }
    else
    {
        // turn search
        // send turn value
        
    }

}




void CPathGenerator::pathCheck()
{
    // check_Matrix.resize(path_Matrix_row);

    // for(int a=0; a<path_Matrix_rows; a++){
    //     check_Matrix[a].resize(path_Matrix_cols);
    // }
    bool checkFlag = false;

    for(int i=0; i<path_Matrix_rows; i++)
    {
        for(int j=0; j<path_Matrix_cols; j++)
        {
            if( gridmap[path_Matrix[i][j].first][path_Matrix[i][j].second] == barrier ){

                path_pub = handle.advertise<burgerking_path_generator::path>("path_msg",10);
                burgerking_path_generator::path pathmsg;   

                pathmsg.distance = 0.0;
                path_pub.publish(pathmsg);
                ROS_INFO("robot don't path following. send path distance = 0.0\n");

                checkFlag = true;
                break;
            }
        }

        if(checkFlag){
            break;
        }
    }


}




void CPathGenerator::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{    
    // gridmap update //
    info = msg->info;
    int rows = info.height;
    int cols = info.width;

    gridmap.resize(rows);
    for (int i=0; i<rows; i++)
    {
        gridmap[i].resize(cols);
    }


    int currCell = 0;
    for( int i=0; i<rows; i++)
    {
        for(int j=0; j<cols; j++)
        {
            if(msg->data[currCell] == -1)
                gridmap[i][j] = unknown;
            else if(msg->data[currCell] == 100)
                gridmap[i][j] = barrier;
            else if(msg->data[currCell] == 0)
                gridmap[i][j] = empty;
            else{}
            currCell++;
        }
    }

//    pathCheck();

//    getRobotState();


    // save gridmap as file //
    static int num = 0;
    std::stringstream name;
    name << "output" << num << ".txt";
    std::ofstream outfile(name.str().c_str());

    for(int i=0; i<rows; i++){
        for(int j=0; j<cols; j++){
            outfile << gridmap[i][j];
        }
        outfile << "\n";
    }
    ROS_INFO("mapCallback finish");
    outfile.close();
    num++;  

}