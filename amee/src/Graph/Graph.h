#ifndef GRAPH_H
#define GRAPH_H

#include "Node.h"
#include "amee/GraphMsg.h"
#include <vector>

namespace amee{

class Graph{
	public:
		Graph();
		Graph(const Graph&);
		~Graph();
		Graph& operator=(const Graph&);

		size_t size() const;
		// void addNode(const Node& n);
		// const Node * getNode(const NodeID) const;
		// void connectNodes(int id1, int id2);

		// const amee::GraphMsg& getGraphMsg();

		// const std::vector<Node*>& getNodes() const;



		//new stuff
		void addNode(const amee::Pose& p, int type, int id);
		int addNode(const amee::Pose& p, int type);

		void addEdges(int id1, int id2);

		Node& getNode(int id);

		amee::GraphMsg getMessage();//reference or pointer or any other thing doesnt matter..
		//end of new stuff

	private:
		std::vector<Node*> mNodes;
		amee::GraphMsg mMsg;

};//class Graph

};//namespace amee

#endif
