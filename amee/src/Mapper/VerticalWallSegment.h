#ifndef VERTICAL_WALLSEGMENT_H
#define VERTICAL_WALLSEGMENT_H

#include "WallSegment.h"
#include "Map.h"

namespace amee {

class VerticalWallSegment : public WallSegment{

	public:
		VerticalWallSegment(Map::Point pos);
		virtual ~VerticalWallSegment();
		/* Adds the given point to this wall if it's close enough. If it was close enough it returns true, otherwise false.	*/
		virtual bool addMeasurement(const Map::Point& pos);
		virtual WallVisualization getVisualization();
		virtual float distanceTo(const Map::Point& pos);
		virtual float getX();
		virtual float getY();
		virtual int getType();
		virtual bool isSmall();
		virtual bool mergeWall(WallSegment* wall);
	private:
		unsigned int mNumberOfPoints;
		float mXAcc;

		bool isInRange(const Map::Point& pos);

	};
}
#endif