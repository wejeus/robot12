#include "Graph.h"
#include <iostream>
#include <fstream>
#include <limits>

using namespace amee;

namespace amee{

Graph::Graph(){
	mCurNodeID = 0;
}

Graph::Graph(const Graph& ref){
	mCurNodeID = ref.mCurNodeID;
	// std::vector<NodeMsg*> v = ref.mNodes;//TODO: fix this part, should be copied or should it?
	mNodes.clear();
	mNodes = ref.mNodes;

	mTagNodes.clear();
	mTagNodes = ref.mTagNodes;

	mRotateLeftNodes.clear();
	mRotateLeftNodes = ref.mRotateLeftNodes;

	mRotateRightNodes.clear();
	mRotateRightNodes = ref.mRotateRightNodes;

	mNextToWallNodes.clear();
	mNextToWallNodes = ref.mNextToWallNodes;
}

// Graph::Graph(const amee::GraphMsg::ConstPtr& gMsg) {
// 	mCurNodeID = gMsg->nodes.size();

// 	mNodes.clear();
// 	std::vector<NodeMsg> &v = gMsg->nodes;
// 	for(std::vector<NodeMsg>::iterator it = v.begin(); it != v.end(); ++it){
// 		mNodes.push_back(new NodeMsg(*it));
// 	}

// 	mTagNodes.clear();
// 	mTagNodes = gMsg->tagNodes;

// 	mRotateLeftNodes.clear();
// 	mRotateLeftNodes = gMsg->rotateLeftNodes;

// 	mRotateRightNodes.clear();
// 	mRotateRightNodes = gMsg->rotateRightNodes;

// 	mNextToWallNodes.clear();
// 	mNextToWallNodes = gMsg->nextToWallNodes;
// }

Graph::~Graph(){
	for (unsigned int i = 0; i < mNodes.size(); ++i) {
		delete mNodes[i];
	}
}

Graph& Graph::operator=(const Graph& ref){
	if(&ref.mNodes != &mNodes){
		this->~Graph();
		new (this) Graph(ref);
	}

	return *this;
}

Graph& Graph::operator=(const amee::GraphMsg::ConstPtr& gMsg) {
	mCurNodeID = gMsg->nodes.size();

	mNodes.clear();
	std::vector<NodeMsg> v = gMsg->nodes;
	for(std::vector<NodeMsg>::iterator it = v.begin(); it != v.end(); ++it){
		mNodes.push_back(new NodeMsg(*it));
	}
	// mNodes = gMsg.nodes;//TODO: fix this part, the pointer might not be valid after the gMsg is destroyed

	mTagNodes.clear();
	mTagNodes = gMsg->tagNodes;

	mRotateLeftNodes.clear();
	mRotateLeftNodes = gMsg->rotateLeftNodes;

	mRotateRightNodes.clear();
	mRotateRightNodes = gMsg->rotateRightNodes;

	mNextToWallNodes.clear();
	mNextToWallNodes = gMsg->nextToWallNodes;

	return *this;
}

size_t Graph::size() const { return mNodes.size(); }

// void Graph::addNode(const Node& n){ mNodes.push_back(new Node(n)); }

// const Node * Graph::getNode(const NodeID id) const { return mNodes[id]; }

const std::vector<NodeMsg*>& Graph::getNodes() const { return mNodes; }

// const amee::GraphMsg& Graph::getGraphMsg() {
// 	unsigned int size = mNodes.size();
// 	mMsg.nodes.resize(size); //TODO fix me
// 	for (unsigned int i = 0; i < mNodes.size(); ++i) {
// 		mMsg.nodes[i] = mNodes[i]->toMsg();
// 	}
// 	return mMsg;
// }

// void Graph::connectNodes(int id1, int id2) {
// 	if ((id1 < 0) || (id1 >= (int)mNodes.size()) || (id2 <0) || (id2 >= (int)mNodes.size())) {
// 		std::cout << "ERROR in Graph::connectNodes(" << id1 << ", " << id2 << ") is invalid." << std::endl;
// 		return;
// 	}
 
// 	mNodes[id1].connectNeighbours(mNodes[id2]);
// }

int Graph::getIDFromPose(const amee::Pose& pose) {
	return getIDFromPose(pose.x, pose.y, pose.theta);
}

int Graph::getIDFromPose(float x, float y, float theta) {
	int best_id_found = -1;
	float best_dist_found = std::numeric_limits<float>::max();

	
	std::vector<NodeMsg*>::const_iterator it;

	float tmpX, tmpY, tmpDist;
	for(it=mNodes.begin(); it != mNodes.end(); ++it) {
		tmpX = (*it)->pose.x;
		tmpY = (*it)->pose.y;

		tmpDist = sqrt((tmpX - x) * (tmpX - x) + (tmpY - y) * (tmpY - y));
		if(tmpDist < MAX_DISTANCE_TO_NODE && tmpDist < best_dist_found){
			best_dist_found = tmpDist;
			best_id_found = (*it)->nodeID;
		}
	}

	return best_id_found;
}

int Graph::getClosestOfType(int type, const amee::Pose& pose) {
	int result = -1;
	switch(type){
		case NODE_NEXT_TO_WALL:		result = findClosestTo(mNextToWallNodes, pose);	break;
		case NODE_ROTATE_LEFT:		result = findClosestTo(mRotateLeftNodes, pose);	break;
		case NODE_ROTATE_RIGHT:		result = findClosestTo(mRotateRightNodes, pose);	break;
		case NODE_TAG:				result = findClosestTo(mTagNodes, pose);			break;
	}
	return result;
}

int Graph::findClosestTo(std::vector<int>& nodeVector, const amee::Pose& pose) {
	int best_id_found = -1;
	float best_dist_found = std::numeric_limits<float>::max();
	for (unsigned int i = 0; i < nodeVector.size(); ++i) {
		float tmpX = mNodes[nodeVector[i]]->pose.x;
		float tmpY = mNodes[nodeVector[i]]->pose.y;

		float tmpDist = sqrt((tmpX - pose.x) * (tmpX - pose.x) + (tmpY - pose.y) * (tmpY - pose.y));
		if (tmpDist < best_dist_found) {
			best_dist_found = tmpDist;
			best_id_found = mNodes[nodeVector[i]]->nodeID;
		}
	}
	return best_id_found;
}

int Graph::addNode(const amee::Pose& p, int type){

	NodeMsg * nMsg_p = new NodeMsg();
	nMsg_p->pose = p;
	nMsg_p->nodeID = mCurNodeID;
	nMsg_p->type = type;

	mNodes.push_back(nMsg_p);

	int tmpCurNodeID = mCurNodeID++;


	switch(type){
		case NODE_NEXT_TO_WALL:		mNextToWallNodes.push_back(tmpCurNodeID);	break;
		case NODE_ROTATE_LEFT:		mRotateLeftNodes.push_back(tmpCurNodeID);	break;
		case NODE_ROTATE_RIGHT:		mRotateRightNodes.push_back(tmpCurNodeID);	break;
		case NODE_TAG:				mTagNodes.push_back(tmpCurNodeID);			break;
	}

	return tmpCurNodeID;
}

void Graph::addEdges(int id1, int id2){

	if (id1 == id2) {
		return;
	}
	
	//add id2 to node with id1
	if(std::find(mNodes[id1]->edges.begin(), mNodes[id1]->edges.end(), id2) != mNodes[id1]->edges.end()){
		//it already contians this id
	}else
		mNodes[id1]->edges.push_back(id2);


	//add id1 to node with id2
	if(std::find(mNodes[id2]->edges.begin(), mNodes[id2]->edges.end(), id1) != mNodes[id2]->edges.end()){
		//it already contians this id
	}else
		mNodes[id2]->edges.push_back(id1);
}

amee::NodeMsg * Graph::getNode(int id){
	return mNodes[id];
}

amee::GraphMsg Graph::getMessage(){
	unsigned int size = mNodes.size();
	GraphMsg graphMsg;

	graphMsg.nodes.resize(size); //TODO fix me
	for (unsigned int i = 0; i < mNodes.size(); ++i) {
		graphMsg.nodes[i] = (*mNodes[i]);
	}

	graphMsg.tagNodes = mTagNodes;
	graphMsg.rotateRightNodes = mRotateRightNodes;
	graphMsg.rotateLeftNodes = mRotateLeftNodes;
	graphMsg.nextToWallNodes = mNextToWallNodes;


	return graphMsg;
}

void Graph::saveToFile(const char * fileName) const {
  std::ofstream graphFile;
  graphFile.open(fileName);
  graphFile << mNodes.size() << ' ';
  for (unsigned int i = 0; i < mNodes.size(); ++i) {
  	NodeMsg* node = mNodes[i];
  	graphFile << node->pose.x << ' ' << node->pose.y << ' ' << node->pose.theta << ' ' << node->nodeID << ' ' << node->type << ' ';
  	graphFile << node->edges.size() << ' ';
  	for (unsigned int j = 0; j < node->edges.size(); ++j) {
  		graphFile << node->edges[j] << ' ';
  	}
  }
  graphFile.close();
}

void Graph::loadFromFile(const char * fileName) {
	std::ifstream graphFile;
	graphFile.open(fileName);
	unsigned int numNodes;
	graphFile >> numNodes;
	mNodes.resize(numNodes);
	mNextToWallNodes.clear();
	mRotateLeftNodes.clear();
	mRotateRightNodes.clear();
	mTagNodes.clear();
	for (unsigned int i = 0; i < numNodes; ++i) {
		NodeMsg* node = new NodeMsg();
		graphFile >> node->pose.x >> node->pose.y >> node->pose.theta >> node->nodeID >> node->type;
		unsigned int numEdges;
		graphFile >> numEdges;
		node->edges.resize(numEdges);
		for (unsigned int j = 0; j  < numEdges; ++j) {
			graphFile >> node->edges[j];
		}
		switch(node->type){
			case NODE_NEXT_TO_WALL:		mNextToWallNodes.push_back(node->nodeID);	break;
			case NODE_ROTATE_LEFT:		mRotateLeftNodes.push_back(node->nodeID);	break;
			case NODE_ROTATE_RIGHT:		mRotateRightNodes.push_back(node->nodeID);	break;
			case NODE_TAG:				mTagNodes.push_back(node->nodeID);			break;
		}
		mNodes[i] = node;
	}
}

};// namespace amee

