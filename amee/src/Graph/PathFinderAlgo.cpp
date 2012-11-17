#include "PathFinderAlgo.h"
#include <iostream>
#include <cmath>

using namespace amee;
using namespace std;

#define LONG_MAX 2147483647
#define BIG_FLOAT 999999999999.9f
#define NODE_ID_UNDEFINED -1

namespace amee{

class CompareNode {
public:
	CompareNode(const Node& source, float * pathDistance):pathD(pathDistance){
		x0 = source.x(); y0 = source.y();
	}
	bool operator()(Node& n1, Node& n2){
		//TODO: check distance to source from both n1 and n2 return true if n1 is less than n2
		if(EuclidDist(n1.x(), n1.y()) < EuclidDist(n2.x(), n2.y()))
			return true;
		return false;
	}
/*
	void setSource(const Node& source){
		mSource = source;
	}
*/
private:
//	Node mSource;
	float x0, y0;
	float * pathD;
	inline float EuclidDist(const float& x1, const float& y1){
		return sqrt(pow((x1 - x0),2) + pow((y1 - y0), 2));
	}
};


/**
 * Calculates the distances from a source to all nodes in the graph.
 */
void PathFinderAlgo::Dijkstra(const Graph& g, const NodeID& source, float const* const* dist, float * pathD, NodeID * path) {
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
/*
		for (i = 0; i < size; ++i)
			if (dist[mini][i])
				if (pathD[mini] + dist[mini][i] < pathD[i]){ 
					pathD[i] = pathD[mini] + dist[mini][i];
					path[i] = mini;
				}
*/
	}
}

};//namespace amee
