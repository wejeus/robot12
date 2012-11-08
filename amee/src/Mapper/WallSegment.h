#ifndef WALLSEGMENT_H
#define WALLSEGMENT_H

#include "Map.h"
#include "amee/WallVisualization.h"

namespace amee {

class WallSegment {

	public:
		
		WallSegment(){};
		virtual ~WallSegment(){};
		/* Adds the given point to this wall if it's close enough. If it was close enough it returns true, otherwise false.	*/
		virtual bool addMeasurement(const Map::Point& pos) = 0;
		// virtual Orientation getOrientation() const = 0;
		virtual WallVisualization getVisualization() = 0;
		// returns the distance from this wall to the given position
		virtual float distanceTo(const Map::Point& pos) = 0;

		virtual int getType() = 0;

		virtual bool isSmall() = 0; 

		/** Tries to merge this wall with the given one. Returns true if merging was successful. If successful the given wall can
		be deleted since this instance will represent the merged wall. */
		virtual bool mergeWall(WallSegment* wall) = 0;

		static const float ORTHOGONAL_TOLERANCE = 0.02f;
		static const float WALL_THICKNESS = 0.015f;
		static const float PARALLEL_TOLERANCE = 0.04f;
		static const int HORIZONTAL = 0;
		static const int VERTICAL = 1;
		static const float SMALL_THRESHOLD = 0.02f;
		static const float SMALL_LENGTH = 0.04f;
		static const float ORTHOGONAL_MERGE_THRESHOLD = 0.03;
		static const float PARALLEL_MERGE_THRESHOLD = 0.09;

	};
}
#endif