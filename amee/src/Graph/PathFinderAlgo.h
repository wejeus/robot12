#ifndef AMEE_PATH_FINDER_ALGO_H
#define AMEE_PATH_FINDER_ALGO_H

#include "Graph.h"
#include "Node.h"

namespace amee{

class PathFinderAlgo{
	public:
		void Dijkstra(const Graph&, const size_t, float const* const*, long *);
		
	private:

};//class PathFinderAlgo

}; // namespace amee

#endif
