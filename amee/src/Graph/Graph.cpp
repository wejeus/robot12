#include "Graph.h"

using namespace amee;

namespace amee{

Graph::Graph(){}

Graph::Graph(const Graph& ref){
	std::vector<Node*> v = ref.getNodes();//TODO: fix this part, should be copied
	mNodes.clear();

	for(std::vector<Node*>::iterator it = v.begin(); it != v.end(); ++it){
		mNodes.push_back(*it);
	}	
}

Graph::~Graph(){
	//TODO:code me
}

Graph& Graph::operator=(const Graph& ref){
	if(&ref.getNodes() != &mNodes){
		this->~Graph();
		new (this) Graph(ref);
	}

	return *this;
}

size_t Graph::size() const { return mNodes.size(); }

void Graph::addNode(Node * n){ mNodes.push_back(n); }

const Node * Graph::getNode(const NodeID id) const { return mNodes[id]; }

const std::vector<Node*>& Graph::getNodes() const { return mNodes; }

};// namespace amee
