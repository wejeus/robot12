#ifndef AMEE_NODE_H
#define AMEE_NODE_H


#include <vector>
#include <map>
#include <stdio.h>

namespace amee{

typedef size_t NodeID;

class Node{
	public:
		enum NODE_TYPE{
			NODE_NEXT_TO_WALL = 1,
			NODE_ROTATE_LEFT = 2, 
			NODE_ROTATE_RIGHT = 4, 
			NODE_TAG = 8
		};

		Node();
		Node(float, float, NodeID);
		~Node();
		Node(const Node&); //copy const
		Node& operator=(const Node&); //assignment operator

		//getters
		const std::vector<NodeID>& getNeighbours() const;
		void getNeighbours(NodeID *) const;
		const std::map<NodeID, float>& getNeighb_dists() const;
		float getDist(const NodeID) const;

		inline float x() const;
		inline float y() const;
		NodeID getID() const;
		const size_t numEdges() const;
		const int getType() const;

		//setters
		inline void x(const float);
		inline void y(const float);
		inline void id(const NodeID);
		void setType(const int);
		void removeType(const int);


		void connectNeighbours(Node&);


		

	private:
		float mX, mY, mAngle;
		int mNODE_STATE;
		NodeID mId;
		std::vector<NodeID> mNeighbours;
		std::map<NodeID, float> mNeighb_dists;

		//for calculating the Euclidean distance between this two nodes
		inline float EuclidDist(const float& x1, const float& x2, const float& y1, const float& y2);

	protected:
		void addNeighbour(const NodeID&);
		void setDist(Node&, const float&);
		
};//class Node

};// namespace amee


#endif