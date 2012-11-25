#ifndef AMEE_PATH_FINDER_ALGO_H
#define AMEE_PATH_FINDER_ALGO_H

#include "Graph.h"
#include <vector>
#include "amee/Pose.h"

namespace amee{

class PathFinderAlgo{
	public:

		static const long mLONG_MAX = 2147483647;
		static const float mBIG_FLOAT = 999999999999.9f;
		static const int mBIG_INTEGER = 9999999;
		static const int mNODE_ID_UNDEFINED = -1;
		static const long NODE_DISTANCE_PENALTY = 0.10; //add this amount of meters in the Dijkstra

		std::vector<amee::Pose> findShortestPath(Graph&, const int startId, const int endId);

		void Dijkstra(Graph& g, const int&, float *, int *);
		
	private:

		inline float EuclidDist(const Pose& p1, const Pose& p2) const;
		inline float EuclidDist(const float& x1, const float& x2, const float& y1, const float& y2) const;

};//class PathFinderAlgo

}; // namespace amee

#endif
