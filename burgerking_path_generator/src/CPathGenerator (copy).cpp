#include "CPathGenerator.h"


CPathGenerator::CPathGenerator():size_r(0.05),searchVertical(5),path_Matrix_rows(0),path_Matrix_cols(0),
old_robotGrid_x(0),old_robotGrid_y(0),linearEq_a(0),linearEq_b(0),linearEq_c(0),
Nd_rows(5),Nd_cols(5),test(1)
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



void CPathGenerator::searchGoal()
{
    goal_Candidate.clear();

    // search goal -- confirm order
    goal_Candidate.push_back(std::make_pair(133,132));
    goal_Candidate.push_back(std::make_pair(133,133));
    goal_Candidate.push_back(std::make_pair(133,134));
    goal_Candidate.push_back(std::make_pair(133,135));
    goal_Candidate.push_back(std::make_pair(133,136));

    GoalNode.H = 0;
    GoalNode.from = goal;

    GoalNode.nodeComponent.clear();
    GoalNode.nodeComponent.resize(Nd_rows);

    for(int i=0; i<Nd_rows; i++){
        GoalNode.nodeComponent[i].resize(Nd_cols);
    }

    ROS_INFO("Goal Node\n");

    for(int m=0; m<Nd_rows; m++){
        for(int n=0; n<Nd_cols; n++){

            GoalNode.nodeComponent[m][n].first = (goal_Candidate[m].first)-(Nd_rows-1)+m; 
            GoalNode.nodeComponent[m][n].second = goal_Candidate[n].second;

            printf("(%d, %d)",GoalNode.nodeComponent[m][n].first,GoalNode.nodeComponent[m][n].second);
        }
        printf("\n");
    }
        
    

}



int CPathGenerator::H_function(Node nd)
{
    int GoalCenter_x = GoalNode.nodeComponent[Nd_rows/2][Nd_cols/2].first;
    int GoalCenter_y = GoalNode.nodeComponent[Nd_rows/2][Nd_cols/2].second;

    int ndCenter_x = nd.nodeComponent[Nd_rows/2][Nd_cols/2].first;
    int ndCenter_y = nd.nodeComponent[Nd_rows/2][Nd_cols/2].second;

    int x_count = abs(GoalCenter_x - ndCenter_x);
    int y_count = abs(GoalCenter_y - ndCenter_y);

    return x_count+y_count;
}



bool CPathGenerator::NodeCapabilityCheck(Node nd)
{
    for(int i=0; i<Nd_rows; i++)
    {
        for(int j=0; j<Nd_cols; j++)
        {
            if( gridmap[nd.nodeComponent[i][j].first][nd.nodeComponent[i][j].second] == barrier ){
                return false;
            }
        }
    }

    return true;

}


bool CPathGenerator::NewNodeCheck(Node nd)
{
    int ndCenter_x = nd.nodeComponent[Nd_rows/2][Nd_cols/2].first;
    int ndCenter_y = nd.nodeComponent[Nd_rows/2][Nd_cols/2].second;

    for( int i=0; i<ClosedNode.size(); i++){
        if( ndCenter_x == ClosedNode[i].nodeComponent[Nd_rows/2][Nd_cols/2].first &&
            ndCenter_y == ClosedNode[i].nodeComponent[Nd_rows/2][Nd_cols/2].second){

                return false;
            }
    }

    return true;

}


Node CPathGenerator::ChooseSelectNode(Node nd1, Node nd2)
{
    if ( nd1.H <= nd2.H){
        return nd1;
    }else{
        return nd2;
    }
}


void CPathGenerator::SetPathNode()
{



}




void CPathGenerator::pathGenerate()
{
    // 1. start node
    // (100,100)is robot position  & robot position is start node
    int rb_x = 100;
    int rb_y = 100;

    StartNode.H = 0;
    StartNode.from = start;

    StartNode.nodeComponent.clear();
    StartNode.nodeComponent.resize(Nd_rows);

    for(int i=0; i<Nd_rows; i++){
        StartNode.nodeComponent[i].resize(Nd_cols);
    }   

    ROS_INFO("Start Node\n");

    for(int m=0; m<Nd_rows; m++){
        for(int n=0; n<Nd_cols; n++){

            StartNode.nodeComponent[m][n].first = rb_x - (Nd_rows/2) + m;
            StartNode.nodeComponent[m][n].second = rb_y - (Nd_cols/2) + n;

            printf("(%d, %d)",StartNode.nodeComponent[m][n].first,StartNode.nodeComponent[m][n].second);
        }
        printf("\n");
    }


    // 2. put start node into closed node
    ClosedNode.push_back(StartNode);


    // 3. parent node = start node
    ParentNode.H = StartNode.H;
    ParentNode.from = StartNode.from;
    ParentNode.nodeComponent = StartNode.nodeComponent;


    while(1)
    {
        // 4. search child node & put into open node
        // (1) up
        Node UpNode;

        UpNode.from = down;

        UpNode.nodeComponent.clear();
        UpNode.nodeComponent.resize(Nd_rows);

        for(int i=0; i<Nd_rows; i++){
            UpNode.nodeComponent[i].resize(Nd_cols);
        }   

        ROS_INFO("Up node\n");

        for(int m=0; m<Nd_rows; m++){
            for(int n=0; n<Nd_cols; n++){

                UpNode.nodeComponent[m][n].first = ParentNode.nodeComponent[m][n].first - (Nd_rows/2);
                UpNode.nodeComponent[m][n].second = ParentNode.nodeComponent[m][n].second;

                printf("(%d, %d)",UpNode.nodeComponent[m][n].first,UpNode.nodeComponent[m][n].second);
            }
            printf("\n");
        }

        UpNode.H = H_function(UpNode);

        if( UpNode.H == 0){
            ROS_INFO("Path found!\n");
            ClosedNode.push_back(UpNode);
            SetPathNode();
            break;
        }

        if(NodeCapabilityCheck(UpNode)){
            if(NewNodeCheck(UpNode)){
            OpenNode.push_back(UpNode);
        }
        }

        // (2) right
        Node RightNode;

        RightNode.from = left;

        RightNode.nodeComponent.clear();
        RightNode.nodeComponent.resize(Nd_rows);

        for(int i=0; i<Nd_rows; i++){
            RightNode.nodeComponent[i].resize(Nd_cols);
        }   

        ROS_INFO("Right node\n");

        for(int m=0; m<Nd_rows; m++){
            for(int n=0; n<Nd_cols; n++){

                RightNode.nodeComponent[m][n].first = ParentNode.nodeComponent[m][n].first;
                RightNode.nodeComponent[m][n].second = ParentNode.nodeComponent[m][n].second + (Nd_cols/2);

                printf("(%d, %d)",RightNode.nodeComponent[m][n].first,RightNode.nodeComponent[m][n].second);
            }
            printf("\n");
        }

        RightNode.H = H_function(RightNode);

        if( RightNode.H == 0){
        ROS_INFO("Path found!\n");
        ClosedNode.push_back(RightNode);
        SetPathNode();
        break;
        }

        if(NodeCapabilityCheck(RightNode)){
            if(NewNodeCheck(RightNode)){
            OpenNode.push_back(RightNode);
        }
        }


        // (3) down
        Node DownNode;

        DownNode.from = up;

        DownNode.nodeComponent.clear();
        DownNode.nodeComponent.resize(Nd_rows);

        for(int i=0; i<Nd_rows; i++){
            DownNode.nodeComponent[i].resize(Nd_cols);
        }   

        ROS_INFO("Down node\n");

        for(int m=0; m<Nd_rows; m++){
            for(int n=0; n<Nd_cols; n++){

                DownNode.nodeComponent[m][n].first = ParentNode.nodeComponent[m][n].first + (Nd_rows/2);
                DownNode.nodeComponent[m][n].second = ParentNode.nodeComponent[m][n].second;

                printf("(%d, %d)",DownNode.nodeComponent[m][n].first,DownNode.nodeComponent[m][n].second);
            }
            printf("\n");
        }

        DownNode.H = H_function(DownNode);

        if( DownNode.H == 0){
        ROS_INFO("Path found!\n");
        ClosedNode.push_back(DownNode);
        SetPathNode();
        break;
        }


        if(NodeCapabilityCheck(DownNode)){
            if(NewNodeCheck(DownNode)){
            OpenNode.push_back(DownNode);
        }
        }


        // (4) left
        Node LeftNode;

        LeftNode.from = right;

        LeftNode.nodeComponent.clear();
        LeftNode.nodeComponent.resize(Nd_rows);

        for(int i=0; i<Nd_rows; i++){
            LeftNode.nodeComponent[i].resize(Nd_cols);
        }   

        ROS_INFO("Left node\n");

        for(int m=0; m<Nd_rows; m++){
            for(int n=0; n<Nd_cols; n++){

                LeftNode.nodeComponent[m][n].first = ParentNode.nodeComponent[m][n].first;
                LeftNode.nodeComponent[m][n].second = ParentNode.nodeComponent[m][n].second - (Nd_cols/2);

                printf("(%d, %d)",LeftNode.nodeComponent[m][n].first,LeftNode.nodeComponent[m][n].second);
            }
            printf("\n");
        }

        LeftNode.H = H_function(LeftNode);

        if( LeftNode.H == 0){
        ROS_INFO("Path found!\n");
        ClosedNode.push_back(LeftNode);
        SetPathNode();
        break;
        }

        if(NodeCapabilityCheck(LeftNode)){
            if(NewNodeCheck(LeftNode)){
            OpenNode.push_back(LeftNode);
        }
        }



        if( OpenNode.size() == 0){
            ROS_INFO("No path!\n");
            break;
        }
        // 5. loop open node & decide parent node (H is the smallest)
        for(int p=0; p<OpenNode.size(); p++)
        {
            if( p == 0){
                SelectNode = OpenNode[0];
            }else{
                SelectNode =  ChooseSelectNode(SelectNode,OpenNode[p]);
            }
        }


        // 6. put into the closed node
        ClosedNode.push_back(SelectNode);

        // 7. parent node = select node 
        ParentNode = SelectNode;

        printf("closed node size = %d\n", (int)ClosedNode.size());
        for(int a=0; a<=ClosedNode.size(); a++){
            for(int m=0; m<Nd_rows; m++){
                for(int n=0; n<Nd_cols; n++){
                printf("(%d, %d)",ClosedNode[a].nodeComponent[m][n].first,ClosedNode[a].nodeComponent[m][n].second);
            }
            printf("\n");
        }

        }


        printf("open node size = %d\n",(int)OpenNode.size());
        for(int a=0; a<=OpenNode.size(); a++){
            for(int m=0; m<Nd_rows; m++){
                for(int n=0; n<Nd_cols; n++){
                printf("(%d, %d)",OpenNode[a].nodeComponent[m][n].first,OpenNode[a].nodeComponent[m][n].second);
            }
            printf("\n");
        }

        }



        ROS_INFO("-------------------------------\n");
    }


}



void CPathGenerator::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{    

    int rows = 192;
    int cols = 192;

    gridmap.resize(rows);
    for (int i=0; i<rows; i++)
    {
        gridmap[i].resize(cols);
    }

    std::ifstream inf("./0_0_5/output52.txt");

    for(int i=0; i<rows; i++){

        char cstr[200];
        //const char* delim = "\n";
        inf.getline(cstr,200,'\n');

        for(int j=0; j<cols; j++){
            //gridmap[i][j] = atoi(&cstr[j]);
            gridmap[i][j] = std::stoi(std::string(&cstr[j], 1));
        }
    }


    // static int num = 0;
    // std::stringstream name;
    // name << "output" << num << ".txt";
    // std::ofstream outfile(name.str().c_str());

    // for(int i=0; i<rows; i++){
    //     for(int j=0; j<cols; j++){
    //         outfile << gridmap[i][j];
    //     }
    //     outfile << "\n";
    // }
    // ROS_INFO("mapCallback finish");
    // outfile.close();
    // num++; 


    // gridmap update //
    // info = msg->info;
    // int rows = info.height;
    // int cols = info.width;

    // gridmap.resize(rows);
    // for (int i=0; i<rows; i++)
    // {
    //     gridmap[i].resize(cols);
    // }


    // int currCell = 0;
    // for( int i=0; i<rows; i++)
    // {
    //     for(int j=0; j<cols; j++)
    //     {
    //         if(msg->data[currCell] == -1)
    //             gridmap[i][j] = unknown;
    //         else if(msg->data[currCell] == 100)
    //             gridmap[i][j] = barrier;
    //         else if(msg->data[currCell] == 0)
    //             gridmap[i][j] = empty;
    //         else{}
    //         currCell++;
    //     }
    // }

    if( test == 1){

    searchGoal();    

    pathGenerate();

    test++;
    }






//    pathCheck();

//    getRobotState();


    // save gridmap as file //
    // static int num = 0;
    // std::stringstream name;
    // name << "output" << num << ".txt";
    // std::ofstream outfile(name.str().c_str());

    // for(int i=0; i<rows; i++){
    //     for(int j=0; j<cols; j++){
    //         outfile << gridmap[i][j];
    //     }
    //     outfile << "\n";
    // }
    // ROS_INFO("mapCallback finish");
    // outfile.close();
    // num++;  

}