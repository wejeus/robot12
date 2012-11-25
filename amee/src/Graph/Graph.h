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

		void addEdges(int id1, int id2);

		NodeMsg* getNode(int id);

		amee::GraphMsg getMessage();

	private:
		size_t mCurNodeID;
		std::vector<NodeMsg*> mNodes;
		std::vector<int> mTagNodes;
		std::vector<int> mRotateLeftNodes;
		std::vector<int> mRotateRightNodes;
		std::vector<int> mNextToWallNodes;
		//amee::GraphMsg mMsg;

};//class Graph

};//namespace amee

#endif
