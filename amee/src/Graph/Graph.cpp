#include "Graph.h"

using namespace amee;

namespace amee{

Graph::Graph(){}

Graph::Graph(const Graph& ref){
	std::vector<Node> v = ref.getNodes();
	mNodes.clear();

	for(std::vector<Node>::iterator it = v.begin(); it != v.end(); ++it){
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

void Graph::addNode(const Node& n){ mNodes.push_back(n); }

Node Graph::getNode(const size_t id) const { return mNodes[id]; }

const std::vector<Node>& Graph::getNodes() const { return mNodes; }

};// namespace amee

