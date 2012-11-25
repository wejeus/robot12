#ifndef AMEE_PATH_FINDER_ALGO_H
#define AMEE_PATH_FINDER_ALGO_H

#include "Graph.h"
#include <vector>
#include "amee/Pose.h"
#include "amee/NodeMsg.h"

namespace amee{

class PathFinderAlgo{
	public:

		static const long mLONG_MAX = 2147483647;
		static const float mBIG_FLOAT = 999999999999.9f;
		static const int mBIG_INTEGER = 9999999;

		static const int mNODE_ID_UNDEFINED = -1;
		static const float MAX_POSITION_DISTANCE = 0.1f;
		static const float MAX_ANGLE_DISTANCE = 10.0f;
		static const long NODE_DISTANCE_PENALTY = 0.10; //add this amount of meters in the Dijkstra

		std::vector<amee::NodeMsg> findShortestPath(Graph&, const int startId, const int endId);
		std::vector<amee::NodeMsg> findShortestPath(Graph&, const float x, const float y, const int endId);
		std::vector<amee::NodeMsg> findShortestPath(Graph&, const float x0, const float y0, const float x1, const float y1);

		void Dijkstra(Graph& g, const int&, float *, int *);

		int getIDfromPose(Graph& g, const float x, const float y) const; /* return -1 if no id is found...*/
		
	private:

		inline float EuclidDist(const Pose& p1, const Pose& p2) const;
		inline float EuclidDist(const float& x1, const float& x2, const float& y1, const float& y2) const;

};//class PathFinderAlgo

}; // namespace amee

#endif
