#include "Node.h"
#include <algorithm>

using namespace amee;

namespace amee{

Node::Node(){}

Node::Node(float _x, float _y, size_t _id):mX(_x),mY(_y),mId(_id){}

Node::Node(const Node& ref){
	mId = ref.getID();
	mX = ref.x();
	mY = ref.y();

	std::vector<size_t> v = ref.getNeighbours();
	mNeighbours.clear();

	for(std::vector<size_t>::iterator it = v.begin(); it != v.end(); ++it){
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

Node::~Node(){
}

const std::vector<size_t>& Node::getNeighbours() const { return mNeighbours; }

inline float Node::x() const { return mX; }

inline float Node::y() const { return mY; }

inline size_t Node::getID() const { return mId; }



inline void Node::x(const float x){ mX = x; }

inline void Node::y(const float y){ mY = y; }

inline void Node::id(const size_t id){ mId = id; }

/**
 * Two way connection
 * Makes both nodes to be each others neighbour
 */ 
void Node::connectNeighbours(Node& other){
	addNeighbour(other.getID());
	other.addNeighbour(mId);
}

/**
 * Adds a nodeID to this nodes neighbour-list.
 */
void Node::addNeighbour(const size_t id){
	if(std::find(mNeighbours.begin(), mNeighbours.end(), id) != mNeighbours.end()){
		//it already contains this id
	}else
		mNeighbours.push_back(id);
}

};//namespace amee
