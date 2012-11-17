#include "Node.h"
#include <algorithm>
#include <cmath>

using namespace amee;

namespace amee{

Node::Node(){}

Node::Node(float _x, float _y, NodeID _id):mX(_x),mY(_y),mId(_id){}

Node::Node(const Node& ref){
	mId = ref.getID();
	mX = ref.x();
	mY = ref.y();

	std::vector<NodeID> v = ref.getNeighbours();
	mNeighbours.clear();

	for(std::vector<NodeID>::iterator it = v.begin(); it != v.end(); ++it){
		mNeighbours.push_back(*it);
	}
}

Node& Node::operator=(const Node& ref){
	if(&ref.getNeighbours() != &mNeighbours){
		this->~Node();
		new (this) Node(ref); //code reuse
	}

	return *this;
}

Node::~Node(){}



const std::vector<NodeID>& Node::getNeighbours() const { return mNeighbours; }

const std::map<NodeID, float>& Node::getNeighb_dists() const { return mNeighb_dists; }

const float Node::getDist(NodeID id) { return mNeighb_dists[id]; }



inline float Node::x() const { return mX; }

inline float Node::y() const { return mY; }

inline NodeID Node::getID() const { return mId; }



inline void Node::x(const float x){ mX = x; }

inline void Node::y(const float y){ mY = y; }

inline void Node::id(const NodeID id){ mId = id; }

/**
 * Two way connection
 * Makes both nodes to be each others neighbour
 */ 
void Node::connectNeighbours(Node& other){
	addNeighbour(other.getID());
	other.addNeighbour(mId);

	float dist = EuclidDist(mX, other.x(), mY, other.y());
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
