#include<iostream>
#include <bits/stdc++.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#define STEP 15
#define GAMMA_FACTOR 25
using namespace std;
using namespace cv;

double D_FACTOR;

struct node
{
        int x,y,parent_index;
        double distance_from_start;
};

vector<node> rrt_tree;  // tree for the RRT
struct vec
{
        double i,j;
};

bool IsValid(Mat img,Point p)       // check if a point is within the image and is not on obstacle
{
        if(p.x<0||p.y<0||p.x>=img.cols||p.y>=img.rows||img.at<uchar>(p.y,p.x) > 0) 
                return 0;
        else
                return 1;
}

double distance(Point p1,Point p2)
{
        return sqrt(pow(p1.x-p2.x,2)+pow(p1.y-p2.y,2));
}

node genRandNode_getNearestTreeNode(Mat map)
{      
        node q;           // generate a random node                       
        do
        {
                q.x = (int) (rand()%map.cols);
                q.y = (int) (rand()%map.rows);
        }
        while(!IsValid(map,Point(q.x,q.y)));  // check if feasible
        double min_dist = FLT_MAX;              
        int min_dist_index = -1;
      //  int point_in_reach = 0;
        for(size_t i = 0;i<rrt_tree.size();i++)              // find the tree-node closest to the random-node
        {
                double dist = distance(Point(q.x,q.y),Point(rrt_tree[i].x,rrt_tree[i].y));      
                if(dist < min_dist)
                {
                        min_dist = dist;
                        min_dist_index = i;   
                }
        }
        q.parent_index = min_dist_index;   // temporary parent is the node closest to it
        q.distance_from_start = rrt_tree[min_dist_index].distance_from_start + min_dist;   // stores the distance of the rand point assuming it to be the parent
        return q;
}

bool find_reachable_node(Mat map,node *rand,node tree_node)
{
       int point_in_reach = 0;
       vec vect;      // initialise a vector in the direction of line joining the two nodes
       vect.i = rand->x - tree_node.x;
       vect.j = rand->y - tree_node.y;
       double vec_magnitude = distance(Point(rand->x,rand->y),Point(tree_node.x,tree_node.y));
       vect.i = vect.i/vec_magnitude;      // get the unit vector along the line
       vect.j = vect.j/vec_magnitude;                          
       for(size_t i = 2;i <= STEP;i++) // search for point in the step size along the line 
       {
               node temp; // node at a distance "i" pixel // i == 1 is tree_node[ ] node itself
               temp.x = tree_node.x + (int)(vect.i*i);
               temp.y = tree_node.y + (int)(vect.j*i);
               if(IsValid(map,Point(temp.x,temp.y)))
               {
                       point_in_reach = 1;        // search till obstacle encountered
                       rand->x = temp.x;
                       rand->y = temp.y;
               } 
               else                       // break if on obstacle the penultimate node is stored in rand node
                       break;
       }
       return point_in_reach;
}

void reWire(vector<int> nodeInRadius,node q)
{
        for(size_t i = 0;i<nodeInRadius.size();i++)
        {
                node *rewired = &rrt_tree[nodeInRadius[i]];
                double dist_frm_start = q.distance_from_start + distance(Point(q.x,q.y),Point(rewired->x,rewired->y));   // new distance from start 
                if(dist_frm_start < rewired->distance_from_start)   // if less than the prev dist
                        rewired->parent_index = rrt_tree.size() - 1;  // the index of q node 
        }
       return ; 
}

vector<int> chooseParent(node *q)
{
        vector<int> node_in_radius;
        double radius_neighbourhood = GAMMA_FACTOR*pow((log(rrt_tree.size())/rrt_tree.size()),D_FACTOR);
        double min_dist = q->distance_from_start;
        for(size_t i = 0;i < rrt_tree.size();i++)
        { 
                double dist = distance(Point(q->x,q->y),Point(rrt_tree[i].x,rrt_tree[i].y));   // the radial distance of a node from q 
                double dist_frm_start = rrt_tree[i].distance_from_start + dist;       // dist from start taking this parent 
                if(dist <= radius_neighbourhood && i != q->parent_index)   //  if node in radius 
                {
                         if(dist_frm_start <  min_dist )   // if dist from start is minimum
                         {
                                 min_dist = dist_frm_start;    // store that dist
                                 q->parent_index = i;      // update parent
                         }
                         node_in_radius.push_back(i);
                }
        }
        q->distance_from_start = min_dist; // set the new min dist
        return node_in_radius; 
}

void tracePath(Mat show_path,node start,node destination)
{
        int not_traced = 1;
        cout<<"Tracing back..."<<endl;
        node child_node = rrt_tree.back();        // to start trace back 
        while(not_traced)        // loop until start is reached
        {
                if(child_node.x == start.x && child_node.y == start.y )         // start is self's parent
                        not_traced = 0; 
                circle(show_path, Point(child_node.x,child_node.y),4,Scalar(0,255,255),-1,LINE_8,0 );
                line(show_path,Point(child_node.x,child_node.y),Point(rrt_tree[child_node.parent_index].x,rrt_tree[child_node.parent_index].y),Scalar(255,0,0),2,LINE_8,0 );
                child_node = rrt_tree[child_node.parent_index];      // parent --> child
        }
        circle(show_path, Point(start.x,start.y),5,Scalar(0,255,0),-1,LINE_8,0 );
        circle(show_path, Point(destination.x,destination.y),5,Scalar(0,0,255),-1,LINE_8,0 );
        return ;
}

void rrt_plan(Mat map_org,node start,node destination)
{
        time_t t;
        double seedval = (unsigned)time(&t);
        Mat map ;
        cvtColor(map_org,map,CV_GRAY2BGR);
        Mat show_path;
        cvtColor(map_org,show_path,CV_GRAY2BGR);
        rrt_tree.push_back(start);  
        int not_reached = 1;       // flag for goal reaching
        srand(seedval);
        while(not_reached)                // loop until not reached
        {
                node q = genRandNode_getNearestTreeNode(map_org);
                if(!find_reachable_node(map_org,&q,rrt_tree[q.parent_index])) continue;   // search for reachable node along the line
                map_org.at<uchar>(q.y,q.x) = 255;   // mark the node as visited
                q.distance_from_start = FLT_MAX;    
                vector<int> node_in_radius = chooseParent(&q);
                circle(map, Point(q.x,q.y),3,Scalar(255,0,0),3,LINE_8,0 );
                line(map,Point(q.x,q.y),Point(rrt_tree[q.parent_index].x,rrt_tree[q.parent_index].y),Scalar(0,150,0),3,LINE_8,0 );
                rrt_tree.push_back(q);
                reWire(node_in_radius,q);
                if(distance(Point(q.x,q.y),Point(destination.x,destination.y)) < STEP)     // if destination is within the reach
                {
                        node temp_dest = destination;
                        if(!find_reachable_node(map_org,&temp_dest,q)) continue;     // check whether node can be connected without tripping over an obstacle
                        if(distance(Point(temp_dest.x,temp_dest.y),Point(destination.x,destination.y)) > 3 ) continue;
                        cout<<"Destination found."<<endl;
                        not_reached = 0;        
                        destination.parent_index = rrt_tree.size()-1;     // parent is the last node pushed
                        rrt_tree.push_back(destination);
                        circle(map, Point(destination.x,destination.y),3,Scalar(255,0,0),3,LINE_8,0 );
                        line(map,Point(q.x,q.y),Point(destination.x,destination.y),Scalar(0,150,0),3,LINE_8,0 );
                }
                imshow("Tree growth",map);
                waitKey(1);
        }
        imshow("Tree growth",map);
        tracePath(show_path,start,destination);
        cout<<"Done... Showing path"<<endl;
        imshow("Path",show_path);
        waitKey(0);
}

int main(int argc,char** argv)
{
        Mat map = imread(argv[1],0);
        D_FACTOR = 1/map.rows*map.cols;
        node start,destination;
        cout<<"Enter the starting point:(row,col) ::"<<endl;
        cin>>start.y>>start.x;
        cout<<"Enter the destination point:(row,col) ::"<<endl;
        cin>>destination.y>>destination.x;

        if((!IsValid(map,Point(start.x,start.y)))||(!IsValid(map,Point(destination.x,destination.y))))
        {
                cout<<"Invalid start or destination"<<endl;
                cout<<"Exiting"<<endl;
                return 0;
        }
        start.parent_index = 0;
        cout<<"Initialising RRT..."<<endl;
        // call rrt to plan path
        rrt_plan(map,start,destination);
        return 0;
}
