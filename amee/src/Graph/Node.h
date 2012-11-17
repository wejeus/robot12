#ifndef AMEE_NODE_H
#define AMEE_NODE_H


#include <vector>
#include <map>
#include <stdio.h>

namespace amee{

typedef size_t NodeID;

class Node{
	public:
		Node();
		Node(float, float, NodeID);
		~Node();
		Node(const Node&); //copy const
		Node& operator=(const Node&); //assignment operator

		//getters
		const std::vector<NodeID>& getNeighbours() const;
		const std::map<NodeID, float>& getNeighb_dists() const;
		const float getDist(NodeID);

		inline float x() const;
		inline float y() const;
		inline NodeID getID() const;

		//setters
		inline void x(const float);
		inline void y(const float);
		inline void id(const NodeID);

		void connectNeighbours(Node&);

	private:
		float mX, mY;
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
