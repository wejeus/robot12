#ifndef GRAPH_H
#define GRAPH_H

#include "amee/NodeMsg.h"
#include "amee/GraphMsg.h"
#include "amee/Pose.h"
#include <vector>

namespace amee{

class Graph{
	public:

		enum NODE_TYPE{
			NODE_NEXT_TO_WALL = 0,
			NODE_ROTATE_LEFT = 1, 
			NODE_ROTATE_RIGHT = 2, 
			NODE_TAG = 3
		};

		static const float MAX_DISTANCE_TO_NODE = 0.1f;

		Graph();
		Graph(const Graph&);
		// Graph(const amee::GraphMsg::ConstPtr&);
		~Graph();
		Graph& operator=(const Graph&);
		Graph& operator=(const amee::GraphMsg::ConstPtr&);

		size_t size() const;
		// void addNode(const Node& n);
		// const Node * getNode(const NodeID) const;
		// void connectNodes(int id1, int id2);

		// const amee::GraphMsg& getGraphMsg();

		const std::vector<NodeMsg*>& getNodes() const;

		// void addNode(const amee::Pose& p, int type, int id);
		int addNode(const amee::Pose& p, int type);

		int getIDFromPose(const amee::Pose& pose);
		int getIDFromPose(float x, float y, float theta);

		int getClosestOfType(int type, const amee::Pose& pose);

		void addEdges(int id1, int id2);

		NodeMsg* getNode(int id);

		amee::GraphMsg getMessage();

		//for serialization
		void saveToFile(const char * fileName) const;
		void loadFromFile(const char * fileName);

	private:
		size_t mCurNodeID;
		std::vector<NodeMsg*> mNodes;
		std::vector<int> mTagNodes;
		std::vector<int> mRotateLeftNodes;
		std::vector<int> mRotateRightNodes;
		std::vector<int> mNextToWallNodes;

		int findClosestTo(std::vector<int>& nodeVector, const amee::Pose& pose);
		//amee::GraphMsg mMsg;

};//class Graph

};//namespace amee

#endif
