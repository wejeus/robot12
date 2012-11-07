#ifndef WALLSEGMENT_H
#define WALLSEGMENT_H

namespace amee {

class WallSegment {

	public:
		enum Orientation {Horizontal, Vertical};
		WallSegment();
		virtual ~WallSegment();
		/* Adds the given point to this wall if it's close enough. If it was close enough it returns true, otherwise false.	*/
		virtual bool addMeasurement(const Map::Point& pos) = 0;
		virtual Orientation getOrientation() const = 0;

		static const float ORTHOGONAL_TOLERANCE = 0.02f;
		static const float WALL_THICKNESS = 0.015f;
		static const float PARALLEL_TOLERANCE = 0.02f;

	};
}
#endif