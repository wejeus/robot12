#include <iostream>
#include "PathFinderAlgo.h"
#include "Graph.h"
#include "amee/NodeMsg.h"
#include "amee/Pose.h"

using namespace std;
using namespace amee;

static const float LL = 999999999.9f;
static const size_t SIZE = 6;

int main(int argc, char ** argv){

	cout << "creating graph" << endl;
	Graph g;

	cout << "Adding nodes to the graph " << endl;

	float correct_dists[] = {0.0f, 7.0f, 10.3078f, 17.0346f, 20.6218f, 13.3417f};
	int correct_path[]  = {0, 0, 0, 2, 5, 0};

	float x_list[] = {0.0f, 7.0f,  2.5f,  7.0f,  5.0f,  3.0f};
	float y_list[] = {0.0f, 0.0f, 10.0f, 15.0f, 20.0f, 13.0f};
	float theta_list[] = {0.0f, 7.0f,  2.5f,  7.0f,  5.0f,  3.0f};
	int type_list[] = {1, 0, 0, 2, 1, 0};
	
	for(size_t i=0; i<SIZE; ++i){
		Pose pose;
		pose.x = x_list[i];
		pose.y = y_list[i];
		pose.theta = theta_list[i];
		g.addNode(pose, type_list[i]);
	}

	//adding edges
	g.addEdges(0,1);
	g.addEdges(0,2);
	g.addEdges(0,5);

	g.addEdges(1,2);
	g.addEdges(1,3);

	g.addEdges(2,3);
	g.addEdges(2,5);

	g.addEdges(3,4);

	g.addEdges(4,5);


	float pathD[SIZE];
	int path[SIZE];
	int source = 0;

	cout << "creating the pathfinder" << endl;
	PathFinderAlgo pf;
	
	cout << "running Dijkstra's algorithm on the graph with source set to " << source << endl;
	pf.Dijkstra(g, source, pathD, path);

	cout << "here are the distances from source(" << source << ") to each node: " << endl;
	for(size_t i=0; i<SIZE; ++i){
		cout << source << " => " << i << ":\t"<< pathD[i] << endl;
		assert(fabs(pathD[i] - correct_dists[i]) < 0.0001f);
	}
	cout << endl;


	cout << "Path to each node starting from source is: " << endl;
	for(size_t i=0; i<SIZE; ++i){
		cout << "from " << path[i] << " to " << i << endl;
		assert(path[i] == correct_path[i]);
	}
	cout << endl;



	//this is how u do for getting the shortest path of positions from a sourceID to a destinationID
	source = 0;
	int dest = 4;
	vector<Pose> vPath = pf.findShortestPath(g, source, dest);
	cout << endl;
	cout << "Here comes the path from " << source << " to " << dest << ":" << endl;
	for(size_t i=0; i<vPath.size(); ++i){
		cout << "(" << vPath[i].x << ", " << vPath[i].y << ")";

		if(i+1 < vPath.size()) cout << " => ";
	}

	cout << endl;


	return 0;
}
