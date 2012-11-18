#include "PathFinderAlgo.h"
#include <iostream>
#include <cmath>
#include <queue>

using namespace amee;
using namespace std;

#define LONG_MAX 2147483647
#define BIG_FLOAT 999999999999.9f
#define BIG_INTEGER 9999999
#define NODE_ID_UNDEFINED -1



class CompareNode {
public:
	CompareNode(float * prioKey):mPrioKey(prioKey){}

	bool operator()(const NodeID& n1, const NodeID& n2) const {
		
		if(mPrioKey[n1] > mPrioKey[n2]){
			return true;
		}
		return false;
	}

private:
	float * mPrioKey;
};

typedef priority_queue<NodeID, vector<NodeID>, CompareNode> NodePQ;

namespace amee{

/**
 * Calculates the distances from a source to all nodes in the graph.
 */
void PathFinderAlgo::Dijkstra(const Graph& g, const NodeID& source, float * pathD, NodeID * path) {
	size_t size = g.size();
	bool visited[size];
	size_t i, k, mini;

	const Node * n;
	const std::vector<NodeID> * v;
	std::vector<NodeID>::const_iterator it;

	//initial values
	for(i=0; i<size; ++i){
		pathD[i] = BIG_FLOAT;
		visited[i] = false;
		path[i] = NODE_ID_UNDEFINED;
	}

	pathD[source] = 0.0f;
	path[source] = 0;



	//Dijkstra
/*
	for (k = 0; k < size; ++k) {
		mini = -1;
		for (i = 0; i < size; ++i)
			if (!visited[i] && ((mini == -1) || (pathD[i] < pathD[mini])))
				mini = i;

		visited[mini] = true;

		n = &(g.getNode(mini));
		v = &(n->getNeighbours());
		for(it = v->begin(); it != v->end(); ++it){
			float dd = n->getDist(*it);

			if(pathD[mini] + dd < pathD[(*it)]){
				pathD[(*it)] = pathD[mini] + dd;
				path[(*it)] = mini;
			}
		}
	}
*/

	//Dijkstra with priority queue

	const CompareNode compareFunc(pathD);
	NodePQ pq(compareFunc); //creating a priority queue with some args to the compare function

	const std::vector<Node*> gNodes = g.getNodes();
	
	std::vector<Node*>::const_iterator n_it;
	for(n_it=gNodes.begin(); n_it != gNodes.end(); ++n_it){
		pq.push((*n_it)->getID());
	}

	NodeID curID;

	while(!pq.empty()){
		curID = pq.top(); pq.pop();

		if(visited[curID]) continue;

		visited[curID] = true;

		if(fabs(pathD[curID] - BIG_FLOAT) < 0.00001)
			break;
		
		v = &(g.getNode(curID)->getNeighbours());
		for(it = v->begin(); it != v->end(); ++it){
			float alt = pathD[curID] + g.getNode(curID)->getDist(*it);

			if(alt < pathD[(*it)]){
				pathD[(*it)] = alt;
				path[(*it)] = curID;
				pq.push((*it));
			}
		}
	}
}

};//namespace amee
