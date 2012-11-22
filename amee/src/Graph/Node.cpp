#include "Node.h"
#include <algorithm>
#include <cmath>

//WARNING: see the headerfile

using namespace amee;
using namespace std;

namespace amee{


Node::Node(){}

Node::Node(amee::Pose p, NodeID _id) {
	mPose = p;
	mId = _id;
}

Node::Node(const Node& ref){
//	cout << "in Node copy constructor" << endl;
	mId = ref.getID();
	mPose = ref.pose();
	mNODE_STATE = ref.mNODE_STATE;

	std::vector<NodeID> v = ref.getNeighbours();
	mNeighbours.clear();

	for(std::vector<NodeID>::iterator it = v.begin(); it != v.end(); ++it){
		mNeighbours.push_back(*it);
		mNeighb_dists[*it] = ref.getDist(*it);
	}
}

Node& Node::operator=(const Node& ref){
//	cout << "in Node operator=" << endl;
	if(&ref.getNeighbours() != &mNeighbours){
		this->~Node();
		new (this) Node(ref); //code reuse
	}

	return *this;
}

Node::~Node(){}



const std::vector<NodeID>& Node::getNeighbours() const { return mNeighbours; }

void Node::getNeighbours(NodeID * list) const {
        for(size_t i=0; i<mNeighbours.size(); ++i){
                list[i] = mNeighbours[i];
        }

}

const std::map<NodeID, float>& Node::getNeighb_dists() const { return mNeighb_dists; }

float Node::getDist(const NodeID id) const{
	if(id == mId){
		return 0.0f;
	}
	
	std::map<NodeID, float>::const_iterator it;
	it = mNeighb_dists.find(id);

	if(it == mNeighb_dists.end()){
		return 2147483647;//long max
	}
	
	return it->second;
}



inline float Node::x() const { return mPose.x; }

inline float Node::y() const { return mPose.y; }

inline float Node::theta() const { return mPose.theta; }

inline amee::Pose Node::pose() const { return mPose; }

NodeID Node::getID() const { return mId; }

int Node::numEdges() const { return mNeighbours.size(); }

int Node::getType() const { return mNODE_STATE; }


inline void Node::x(const float x){ mPose.x = x; }

inline void Node::y(const float y){ mPose.y = y; }

inline void Node::theta(const float theta) { mPose.theta = theta; }

inline void Node::id(const NodeID id){ mId = id; }

void Node::setType(const int t) { mNODE_STATE = t; }

amee::NodeMsg Node::toMsg() const {
	NodeMsg msg;
	msg.pose = mPose;
	msg.nodeID = mId;
	for (unsigned int i = 0; i < mNeighbours.size(); ++i) {
		msg.edges[i] = mNeighbours[i];
	}
	return msg;
}


/**
 * Two way connection
 * Makes both nodes to be each others neighbour
 */ 
void Node::connectNeighbours(Node& other){
	addNeighbour(other.getID());
	other.addNeighbour(mId);

	float dist = EuclidDist(mPose.x, other.x(), mPose.y, other.y());
	setDist(other, dist);
	other.setDist(*this, dist);
}

inline float Node::EuclidDist(const float& x1, const float& x2, const float& y1, const float& y2){
	return sqrt(pow(x1-x2,2) + pow(y1-y2, 2));
}


/**
 * Adds a nodeID to this nodes neighbour-list.
 */
void Node::addNeighbour(const NodeID& id){
	if(std::find(mNeighbours.begin(), mNeighbours.end(), id) != mNeighbours.end()){
		//it already contains this id
	}else
		mNeighbours.push_back(id);
}

void Node::setDist(Node& other, const float& dist){
	mNeighb_dists[other.getID()] = dist;
}

};//namespace amee
