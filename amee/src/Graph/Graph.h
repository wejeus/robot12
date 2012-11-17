#ifndef GRAPH_H
#define GRAPH_H

#include "Node.h"
#include <vector>

namespace amee{

class Graph{
	public:
		Graph();
		Graph(const Graph&);
		~Graph();
		Graph& operator=(const Graph&);

		size_t size() const;
		void addNode(const Node&);
		const Node& getNode(const NodeID) const;

		const std::vector<Node>& getNodes() const;

	private:
		std::vector<Node> mNodes;

};//class Graph

};//namespace amee

#endif
