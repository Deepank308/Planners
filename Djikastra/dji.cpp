#include<iostream>
#include<bits/stdc++.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#define VALID 1

using namespace std;
using namespace cv;

time_t tic,toc;

struct node                    // to store info of a node
{
        Point coord;
        Point parent;
        double dist;           // distance from start
};

bool operator<(const node &n1,const node &n2) // for priority queue
{
        return n1.dist>n2.dist;
}

bool IsValid(Mat img, Point P)    // check if pixel is in image and not on obstacle
{
        if(P.x<0||P.y<0||P.x>=img.cols||P.y>=img.rows||img.at<uchar>(P.y,P.x) == 255)
                return !VALID;
        else
                return VALID;
}

double distance(node n1,node n2)
{
        return sqrt(pow(n1.coord.x-n2.coord.x,2)+pow(n1.coord.y-n2.coord.y,2));
}

void dji_plan(Mat map,Point start,Point dest)  // x  ---> cols   y ---> rows   i ---> cols  j ---> rows
{
        Mat show_traversal = map.clone(),final_path = map.clone();
        priority_queue<node> Q; 
        node init;   // to initialize the nodes vector with each node at INF
        init.dist = FLT_MAX;
        vector<vector<node> >  nodes(map.rows,vector<node> (map.cols,init));  // contains info about the nodes
        node start_node;  // declare the start node to push into priority_queue
        start_node.coord = Point(start.x,start.y);  
        start_node.parent = Point(start.x,start.y); 
        start_node.dist = 0; 
        nodes[start.y][start.x].parent = Point(start.x,start.y); 
        nodes[start.y][start.x].dist = 0;
        Q.push(start_node);  // push into the Queue
        int flag_reached = 0;
        cout<<"Initialising  the search"<<endl;
        while(!Q.empty())  
        {
                node curr_parent = Q.top();    // pop the top element
                Q.pop();
                show_traversal.at<uchar>(curr_parent.coord.y,curr_parent.coord.x) = 255;
                for(int i = curr_parent.coord.x-1;i<curr_parent.coord.x+2;i++)      // iterate in a kernel 3X3
                {
                        for(int j = curr_parent.coord.y-1;j<curr_parent.coord.y+2;j++)
                        {
                                if(!IsValid(map,Point(i,j))) continue;      // if not in image or on an obstacle       
                                nodes[j][i].coord = Point(i,j);                                                  
                                double cost = distance(curr_parent,nodes[j][i]);                                                  
                                if(cost + nodes[curr_parent.coord.y][curr_parent.coord.x].dist < nodes[nodes[j][i].coord.y][nodes[j][i].coord.x].dist)// distance condition 
                                {
                                        nodes[j][i].parent = Point(curr_parent.coord.x,curr_parent.coord.y);
                                        nodes[j][i].dist = nodes[curr_parent.coord.y][curr_parent.coord.x].dist + cost; // update the distance from start
                                        Q.push(nodes[j][i]); 
                                }
                                if(nodes[j][i].coord == Point(dest.x,dest.y)) // if dest is reached
                                {
                                        cout<<"Destination Reached"<<endl;
                                        flag_reached = 1;
                                        break;
                                }   
                        }
                        if(flag_reached)
                                break;
                }
                if(flag_reached)
                        break;
               // namedWindow("Traversal",WINDOW_NORMAL);
               // imshow("Traversal", show_traversal);
              //  waitKey(1);
        }
        cout<<"Tracing back the path"<<endl;
        int traced = 0;  // flag for path tracing
        Point child = Point(nodes[dest.y][dest.x].coord.x,nodes[dest.y][dest.x].coord.y);
        while(!traced)
        {
            Point parent = Point(nodes[child.y][child.x].parent.x,nodes[child.y][child.x].parent.y);
            if(parent == Point(child.x,child.y))  // for only start_node parent == child
                    traced = 1;
            final_path.at<uchar>(child.y,child.x) = 255;     // draw path
            child = Point(parent.x,parent.y); // set parent --> child 
            // imshow("Path",final_path);
            // waitKey(1);
        }
        cout<<"Traced"<<endl;
        cout<<"Showing Path"<<endl;
        cout<<"Cost of the shown path :: "<<nodes[dest.y][dest.x].dist<<endl;
        imshow("Path",final_path);
        waitKey(0);
        return;
}

int main(int argc, char **argv)
{
        Mat map = imread(argv[1],0);
        Point start,destination;
        cout<<"Enter the starting coordinates : "<<endl;
        cin >>start.y>>start.x;
        cout<<"Enter the destination coordinates : "<<endl;
        cin>>destination.y>>destination.x;

        // check if start or dest is valid or not
        if((!IsValid(map,start))||(!IsValid(map,destination)))
        {
                cout<<"Invalid Start or Goal"<<endl;
                cout<<"Exiting"<<endl;
                return 0;
        }
        tic = clock();
        // call dji to make path
        dji_plan(map,start,destination);
        toc = clock() - tic;
        double sec_elapsed = (double)toc/CLOCKS_PER_SEC;
        cout<<"Time elapsed :: "<<sec_elapsed<<" secs"<<endl;

        return 0;
}
