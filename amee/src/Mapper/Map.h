#ifndef MAP_H
#define MAP_H

#include <vector> 
#include "ros/ros.h"

namespace amee {

class WallSegment;

class Map {

	public:
		struct Point{
			float x;
			float y;
			const Point operator+(const Point& b) const {
				Point r;
				r.x = x + b.x;
				r.y = y + b.y;
				return r;
			}
			void rotate(float angle) {
				float tx = x;
				float ty = y;
				x = cos(angle) * tx - sin(angle) * ty;
				y = sin(angle) * tx + cos(angle) * ty;
			}
		};
		void addMeasurement(Point pos);
		void print();
		Map();
		~Map();
	
	private:
		std::vector<WallSegment*> mWalls;

	};
}
#endif