#ifndef AMEE_NODE_H
#define AMEE_NODE_H


#include <vector>
#include <stdio.h>

namespace amee{

class Node{
	public:
		Node();
		Node(float, float, size_t);
		~Node();
		Node(const Node&); //copy const
		Node& operator=(const Node&); //assignment operator
		const std::vector<size_t>& getNeighbours() const;
		inline float x() const;
		inline float y() const;
		inline size_t getID() const;

		inline void x(const float);
		inline void y(const float);
		inline void id(const size_t);
		void connectNeighbours(Node&);
		void addNeighbour(const size_t);

	private:
		float mX, mY;
		size_t mId;
		std::vector<size_t> mNeighbours;
		
};//class Node

};// namespace amee


#endif
