#ifndef WALLSEGMENT_H
#define WALLSEGMENT_H

#include "Map.h"
#include "amee/WallVisualization.h"

namespace amee {

class WallSegment {
	protected:
		Map::Point mFrom, mTo;

		/* Adds the given point to this wall. Call this only if the measurement belongs to this wall!*/
		virtual bool addMeasurement(const Map::Point& pos) = 0;

		// Return true if the line segments (fromA, toA) and (fromB, toB) intersect. If so t and s describe the intersection point such as
		// intersection = fromA + t * (toA - fromA)
		// and 
		// intersection = fromB + s * (toB - fromB)
		bool intersect(const Map::Point& fromA, const Map::Point& toA, const Map::Point& fromB, const Map::Point& toB, Map::Point& intersection, float& t, float& s) {
			Map::Point dirA = toA - fromA;
			Map::Point dirB = toB - fromB;
			float det = dirB.y * dirA.x - dirB.x * dirA.y; 
			if (fabs(det) <= 0.001f) return false; // determinant is almost 0 -> lines are parallel 
			Map::Point p = fromA - fromB;
			s = 1.0f/det * (dirA.y * (-1.0f * p.x) + dirA.x * p.y);
			t =  1.0f/det * (dirB.y * (-1.0f * p.x) + dirB.x * p.y);
			intersection = fromB + dirB * s; 
			return ((0.0f <= t) && (t <= 1.0f) && (0.0f <= s) && (s <= 1.0f));
		}
	

	public:
		static const float ORTHOGONAL_TOLERANCE = 0.02f;
		static const float WALL_THICKNESS = 0.015f;
		static const float PARALLEL_TOLERANCE = 0.04f;
		static const int HORIZONTAL = 0;
		static const int VERTICAL = 1;
		static const int NONE = 2;
		static const float SMALL_THRESHOLD = 0.02f;
		static const float SMALL_LENGTH = 0.035f;
		static const float ORTHOGONAL_MERGE_THRESHOLD = 0.05f;
		static const float PARALLEL_MERGE_THRESHOLD = 0.10f;
		static const float ERROR_TOLERANCE = 0.05f;

		WallSegment(){};
		virtual ~WallSegment(){};

		// virtual Orientation getOrientation() const = 0;
		virtual WallVisualization getVisualization() = 0;
		// returns the distance from this wall to the given position
		virtual float distanceTo(const Map::Point& pos) = 0;

		// Return coordinates of wall center.
		virtual float getX() = 0;
		virtual float getY() = 0;

		virtual int getType() = 0;

		virtual bool isSmall() = 0; 

		/** 	
		*/
		bool belongsToWall(const Map::Point& sensor, const Map::Point& measurement, Map::Point& intersection, float& t) {
			Map::Point dir = (measurement - sensor).normalized();
			Map::Point errorMeasurement = measurement + dir * 0.04f;// set as constant, but linker doesn't allow that oO
			Map::Point errorSensor = sensor - dir * 0.11f; // this is in the middle of the robot. This is to even detect a wall if we have a huge error in our position estimation. 
			if (getType() == HORIZONTAL) {
				dir.x = 1.0f;
				dir.y = 0.0f;
			} else {
				dir.x = 0.0f;
				dir.y = 1.0f;
			}
			Map::Point errorWallFrom = mFrom - dir * 0.01f;
			Map::Point errorWallTo = mTo + dir * 0.01f;
			float s;
			bool associate = intersect(errorSensor, errorMeasurement, errorWallFrom, errorWallTo, intersection, t, s);
			return associate;
		}

		bool mapMeasurement(const Map::Point& sensor, const Map::Point& measurement, Map::Point& intersection) {
			float t = 0.0f;
			if (belongsToWall(sensor, measurement, intersection, t)) {
				addMeasurement(intersection);
				return true;
			}
			return false;
		}

		/** Tries to merge this wall with the given one. Returns true if merging was successful. If successful the given wall can
		be deleted since this instance will represent the merged wall. */
		virtual bool mergeWall(WallSegment* wall) = 0;
	};	
}
#endif