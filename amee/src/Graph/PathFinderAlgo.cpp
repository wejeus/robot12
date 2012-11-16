#include "PathFinderAlgo.h"

using namespace amee;

#define LONG_MAX 2147483647

namespace amee{

/**
 * Calculates the distances from a source to all nodes in the graph.
 * It returns the distance from source to the i-th Node where i == NodeID.
 * The last arg is a matrix with distances from node i to node j.
 */
void PathFinderAlgo::Dijkstra(const Graph& g, size_t source, float const* const* dist, long * pathD) {
	//TODO: implement dijkstra
	size_t size = g.size();
	bool visited[size];
//	long pathD[size]; //path distance, 
	size_t i, k, mini;

	//initial values
	for(i=0; i<size; ++i){
		pathD[i] = LONG_MAX;
		visited[i] = false;
	}

	pathD[source] = 0;

	//Dijkstra

	for (k = 0; k < size; ++k) {
		mini = -1;
		for (i = 0; i < size; ++i)
			if (!visited[i] && ((mini == -1) || (pathD[i] < pathD[mini])))
				mini = i;

		visited[mini] = true;

		for (i = 0; i < size; ++i)
			if (dist[mini][i])
				if (pathD[mini] + dist[mini][i] < pathD[i]) 
					pathD[i] = pathD[mini] + dist[mini][i];
	}
}

};//namespace amee
