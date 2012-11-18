#include <iostream>
#include "PathFinderAlgo.h"
#include "Graph.h"
#include "Node.h"

using namespace std;
using namespace amee;

static const float LL = 999999999.9f;
static const size_t SIZE = 6;
/*float dists[SIZE][SIZE] = {{ 0.0f, 7.0f, 9.0f,LL,LL,14.0f},
                           { 7.0f, 0.0f, 10.0f,15.0f,LL,LL},
                           { 9.0f,10.0f, 0.0f,11.0f,LL, 2.0f},
                           {LL,15.0f,11.0f, 0.0f, 6.0f,LL},
                           {LL,LL,LL, 6.0f, 0.0f, 9.0f},
                           {14.0f,LL, 2.0f,LL, 9.0f, 0.0f}};
*/

int main(int argc, char ** argv){

	cout << "creating graph" << endl;
	Graph g;

	cout << "creating nodes" << endl;

	Node * nList = new Node[SIZE];

	float x_list[] = {0.0f, 7.0f,  2.5f,  7.0f,  5.0f,  3.0f};	
	float y_list[] = {0.0f, 0.0f, 10.0f, 15.0f, 20.0f, 13.0f};
	
	for(size_t i=0; i<SIZE; ++i){
		nList[i] = Node(x_list[i], y_list[i], i);
	}

	

	nList[0].connectNeighbours(nList[1]);
	nList[0].connectNeighbours(nList[2]);
	nList[0].connectNeighbours(nList[5]);

	nList[1].connectNeighbours(nList[2]);
	nList[1].connectNeighbours(nList[3]);

	nList[2].connectNeighbours(nList[3]);
	nList[2].connectNeighbours(nList[5]);

	nList[3].connectNeighbours(nList[4]);

	nList[4].connectNeighbours(nList[5]);

	
	vector<NodeID>::const_iterator it;
	for(size_t i=0; i<SIZE; ++i){
		vector<NodeID> v = nList[i].getNeighbours();
		for(it = v.begin(); it != v.end(); ++it){
			cout << "Dist: " << i << " to " << (*it) << " == " << nList[i].getDist((*it)) << endl;
//			cout << "And : " << (*it) << " to " << i << " == " << nList[(*it)].getDist(i) << endl;
//			cout << endl;
		}
		cout << "##############################" << endl;
		
	}


	cout << "\n\nSetting type of 0 to NODE_NEXT_TO_WALL and NODE_TAG" << endl;
	nList[0].setType(Node::NODE_NEXT_TO_WALL | Node::NODE_TAG);
	cout << nList[0].getType() << " == " << (Node::NODE_NEXT_TO_WALL | Node::NODE_TAG) << "?" << endl;
	cout << "Removing the NODE_NEXT_TO_WALL" << endl;
	nList[0].removeType(Node::NODE_NEXT_TO_WALL);
	cout << nList[0].getType() << " == " << Node::NODE_TAG << "?"<< endl;


	cout << "adding the nodes to the graph" << endl;
	for(size_t i=0; i<SIZE; ++i){
		g.addNode(&nList[i]);
	}



	float pathD[SIZE];
	NodeID path[SIZE];
	NodeID source = 0;

	cout << "creating the pathfinder" << endl;
	PathFinderAlgo pf;
	
	cout << "running Dijkstra's algorithm on the graph with source set to " << source << endl;
	pf.Dijkstra(g, source, pathD, path);

	cout << "here are the distances from source(" << source << ") to each node: " << endl;
	for(size_t i=0; i<SIZE; ++i){
		cout << source << " => " << i << ":\t"<< pathD[i] << endl;
	}
	cout << endl;


	cout << "Path to each node starting from source is: " << endl;
	for(size_t i=0; i<SIZE; ++i){
		cout << "from " << path[i] << " to " << i << endl;
	}
	cout << endl;

	cout << "cleaning up" << endl;
/*	for(size_t i=0; i<SIZE; ++i){
		delete [] dists[i];
	}
	delete [] dists;*/
	delete [] nList;

	return 0;
}
