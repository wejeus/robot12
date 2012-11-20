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
	for (unsigned int i = 0; i < mNodes.size(); ++i) {
		delete mNodes[i];
	}
}

Graph& Graph::operator=(const Graph& ref){
	if(&ref.getNodes() != &mNodes){
		this->~Graph();
		new (this) Graph(ref);
	}

	return *this;
}

size_t Graph::size() const { return mNodes.size(); }

void Graph::addNode(const Node& n){ mNodes.push_back(new Node(n)); }

const Node * Graph::getNode(const NodeID id) const { return mNodes[id]; }

const std::vector<Node*>& Graph::getNodes() const { return mNodes; }

const amee::GraphMsg& Graph::getGraphMsg() {
	unsigned int size = mNodes.size();
	mMsg.nodes.resize(size); //TODO fix me
	for (unsigned int i = 0; i < mNodes.size(); ++i) {
		mNodes[i]->toMsg(mMsg.nodes[i]);
	}
	return mMsg;
}

void Graph::connectNodes(int id1, int id2) {
	if ((id1 < 0) || (id1 >= (int)mNodes.size()) || (id2 <0) || (id2 >= (int)mNodes.size())) {
		std::cout << "ERROR in Graph::connectNodes(int id1, int id2): id1:" << id1 << " or id2:" << id2 << " is invalid." << std::endl;
		return;
	}

	if (id1 == id2) {
		return;
	}
	// unsigned int u_id1 = (unsigned int)id1;

	// std::cout << "connecting neighbors " << id1 << " " << id2 << std::endl;
	mNodes[id1]->connectNeighbours(mNodes[id2]);
}

};// namespace amee

