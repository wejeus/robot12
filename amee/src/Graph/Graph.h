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
		void addNode(const Node& n);
		const Node * getNode(const NodeID) const;
		void connectNodes(int id1, int id2);

		const amee::GraphMsg& getGraphMsg() const;

		const std::vector<Node*>& getNodes() const;

	private:
		std::vector<Node*> mNodes;
		amee::GraphMsg mMsg;

};//class Graph

};//namespace amee

#endif
