#ifndef AMEE_PATH_FINDER_ALGO_H
#define AMEE_PATH_FINDER_ALGO_H

#include "Graph.h"
#include "Node.h"

namespace amee{

class PathFinderAlgo{
	public:
		void Dijkstra(const Graph&, const NodeID&, float *, NodeID *);
		
	private:

};//class PathFinderAlgo

}; // namespace amee

#endif
