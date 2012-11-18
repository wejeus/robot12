#ifndef MAP_H
#define MAP_H

#include <list> 
#include "ros/ros.h"
#include "amee/MapVisualization.h"

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
		void getVisualization(MapVisualization& vis);
		/** Tries to reduce the number of walls by unifying walls that are close to the given pos (if the seem to be one wall)
			and removes small walls that are far from the given pos. 
		*/
		void reduceNumWalls(const Point& pos, float distance);
		Map();
		~Map();
	
	private:
		std::list<WallSegment*> mWalls;

	};
}
#endif