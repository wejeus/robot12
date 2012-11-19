#ifndef MAP_H
#define MAP_H

#include <list> 
#include "ros/ros.h"
#include "amee/MapVisualization.h"
#include "amee/Pose.h"

namespace amee {

class WallSegment;

class Map {

	public:
		struct Point{
			float x;
			float y;

			Point() : x(0.0f),y(0.0f) {	}

			Point(float tx, float ty) {
				x = tx;
				y = ty;
			}

			const Point operator+(const Pose& p) const {
				Point r;
				r.x = x + p.x;
				r.y = y + p.y;
				return r;
			}

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
		// Adds the given measurement to an existing wall if possible. If not possible, it creates a new wall of type newWallType
		// at position pos. The returned WallSegment is either the associated wall segment or null if no association worked.
		WallSegment* addMeasurement(const Point& pos, int newWallType);
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