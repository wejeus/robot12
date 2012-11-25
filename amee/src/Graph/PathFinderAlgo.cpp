#include "PathFinderAlgo.h"
#include <iostream>
#include <cmath>
#include <queue>
#include <algorithm>

using namespace amee;
using namespace std;


namespace amee{


/**
 * This CompareNode class is for the Dijkstra's algorithms priority queue
 */
class CompareNode {
	public:
		CompareNode(float * prioKey):mPrioKey(prioKey){}

		bool operator()(const int& n1, const int& n2) const {
			
			if(mPrioKey[n1] > mPrioKey[n2]){
				return true;
			}
			return false;
		}

	private:
		float * mPrioKey;
};

typedef priority_queue<int, vector<int>, CompareNode> NodePQ;

/**
 * This gives you a shortest path of from a starting node ID to an end ID in a graph.
 * The return value is a vector of Pose in ordered way.
 */
vector<Pose> PathFinderAlgo::findShortestPath(Graph& g, const int startId, const int endId){
	size_t size = g.size();
	float pathD[size];
	int path[size];

	Dijkstra(g, startId, pathD, path);

	int curDest = endId;
	vector<Pose> v;

	while(curDest != startId){
		// cout << "from " << path[curDest] << " to " << curDest << endl;
		v.push_back(g.getNode(curDest)->pose);
		curDest = path[curDest];
	}

	reverse(v.begin(), v.end());

	return v;
}

vector<Pose> PathFinderAlgo::findShortestPath(Graph& g, const float x, const float y, const int endId) {
	int curID = getIDfromPose(g, x, y);
	vector<Pose> v;
	if( curID == -1){
		std::cout << "couldn't find curID in findShortestPath(Graph, float, float, int)" << std::endl;
		return v;
	}

	return findShortestPath(g, curID, endId);
}


/**
 * Calculates the distances from a source to all nodes in the graph.
 */
void PathFinderAlgo::Dijkstra(Graph& g, const int& source, float * pathD, int * path) {
	size_t size = g.size();
	bool visited[size];
	size_t i;

	const std::vector<unsigned int> * v;
	std::vector<unsigned int>::const_iterator it;

	//initial values
	for(i=0; i<size; ++i){
		pathD[i] = mBIG_FLOAT;
		visited[i] = false;
		path[i] = mNODE_ID_UNDEFINED;
	}

	pathD[source] = 0.0f;
	path[source] = 0;



	//Dijkstra
/*
	size_t k, mini;
	const NodeMsg * n;

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

	const std::vector<NodeMsg*> gNodes = g.getNodes();
	
	std::vector<NodeMsg*>::const_iterator n_it;
	for(n_it=gNodes.begin(); n_it != gNodes.end(); ++n_it){
		pq.push((*n_it)->nodeID);
	}

	int curID;

	while(!pq.empty()){
		curID = pq.top(); pq.pop();

		if(visited[curID]) continue;

		visited[curID] = true;

		if(fabs(pathD[curID] - mBIG_FLOAT) < 0.00001)
			break;
		
		v = &(g.getNode(curID)->edges);
		for(it = v->begin(); it != v->end(); ++it){
			float alt = pathD[curID] + EuclidDist(g.getNode(curID)->pose, g.getNode(*it)->pose);

			if(alt < pathD[(*it)]){
				pathD[(*it)] = alt;
				path[(*it)] = curID;
				pq.push((*it));
			}
		}
	}
}

int PathFinderAlgo::getIDfromPose(Graph& g, const float x, const float y) const {
	int best_id_found = -1;
	float best_dist_found = mBIG_FLOAT;

	const std::vector<NodeMsg*> gNodes = g.getNodes();
	std::vector<NodeMsg*>::const_iterator it;

	float tmpX, tmpY, tmpDist;
	for(it=gNodes.begin(); it != gNodes.end(); ++it) {
		tmpX = (*it)->pose.x;
		tmpY = (*it)->pose.y;

		tmpDist = EuclidDist(tmpX, x, tmpY, y);
		if(tmpDist < MAX_POSITION_DISTANCE && tmpDist < best_dist_found){
			best_dist_found = tmpDist;
			best_id_found = (*it)->nodeID;
		}
	}

	return best_id_found;
}


inline float PathFinderAlgo::EuclidDist(const Pose& p1, const Pose& p2) const {
	return EuclidDist(p1.x, p2.x, p1.y, p2.y);
}

inline float PathFinderAlgo::EuclidDist(const float& x1, const float& x2, const float& y1, const float& y2) const {
	return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2));
}

};//namespace amee
