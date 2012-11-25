#ifndef MAP_H
#define MAP_H

#include <list> 
#include "ros/ros.h"
#include "amee/MapVisualization.h"
#include "amee/Pose.h"
#include <iostream>

namespace amee {

class WallSegment;
class HorizontalWallSegment;
class VerticalWallSegment;

class Map {

	friend std::ostream& operator<< (std::ostream &out, const Map& m);

	public:
		struct Point{
			float x;
			float y;

			Point() : x(0.0f),y(0.0f) {	}

			Point(float tx, float ty) {
				x = tx;
				y = ty;
			}

			Point(amee::Pose pose) {
				x = pose.x;
				y = pose.y;
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

			const Point operator-(const Point& b) const {
				Point r;
				r.x = x - b.x;
				r.y = y - b.y;
				return r;
			}

			const Point operator*(const float& t) const {
				Point r;
				r.x = t * x;
				r.y = t * y;
				return r;
			}

			float operator*(const Point& b) const {
				return x * b.x + y * b.y;
			}

			const Point normalized() const {
				float l = length();
				Point r(x/l, y/l);
				return r;
			}
			float length() const {
				return sqrt(x*x + y*y);
			}

			void rotate(float angle) {
				float tx = x;
				float ty = y;
				x = cos(angle) * tx - sin(angle) * ty;
				y = sin(angle) * tx + cos(angle) * ty;
			}
		}; // Point

		struct WallMatching
		{
			float t; // describes how close the intersection is to the robot t = 0 - inside robot; t = 1 - at the measurementPos
			WallSegment* wall;
		};

		struct Measurement {
			bool valid;// stores if measurement is valid or not (within a certain range)
			Point pos; // measurements position in world coordinates
			Point sensorPos; // sensors position in world coordinates
			float dist; // actual measurement
			Point sensorRelativePos; // sensors position relative to robot center
		};

		struct MeasurementSet {
			Measurement leftFront;
			Measurement leftBack;
			Measurement rightFront;
			Measurement rightBack;
		};

		// Adds the given measurement to an existing wall if possible. If not possible, it creates a new wall of type newWallType
		// at position measurement. Returns true if a wall was found and no new one had to be created,
		// false if none of the existing walls matches.
		// If the measurement is associated with a wall the intersection between the ray (sensor, measurement) and the wall is stored
		// in intersection. sensor describes the position where the sensor is that measured a wall at measurement.
		// bool addMeasurement(const Point& sensor, const Point& measurement, Point& intersection, int newWallType);

		// Adds the given measurement to a wall if possible. If no wall was found and createWall is true, a new wall is initialized 
		// with the given measurement. If createWall is false, the measurement is dropped.
		void addMeasurement(const Measurement& m, int newWallType);
		
		// Localizes the robot based on the given pose and measurements in the map. If localizing is not successfull outPose = inPose
		void localize(const amee::Pose& inPose, const MeasurementSet& measurements, amee::Pose& outPose, bool left, bool right);
		void print();
		void getVisualization(MapVisualization& vis);
		/** Tries to reduce the number of walls by unifying walls that are close to the given pos (if the seem to be one wall)
			and removes small walls that are far from the given pos. 
		*/
		void reduceNumWalls(const Point& pos, float distance);

		void readFromStream(std::istream& in);
		void deleteWalls();

		// Returns true if there is a linear collision free path between start and end. buffer describes an additional
		// buffer distance the robot is supposed to have to any obstacle. robotRadius is the actual radius of the robot.
		// Please note that buffer must be greater 0. Otherwise no path will be found.
		bool isPathCollisionFree(const Point& start, const Point& end, float buffer, float robotRadius);

		float getDistanceToClosestWall(const Point& p);

		Map();
		~Map();


	
	private:
		std::list<HorizontalWallSegment*> mHorizontalWalls;
		std::list<VerticalWallSegment*> mVerticalWalls;
		WallSegment* findBestMatch(const Measurement& m, Point& intersection);
		float moveAngleToInterval02PI(float theta);
		float getWallAngle(const Pose& pose, WallSegment* wall, bool left);
		void insertHorizontal(HorizontalWallSegment* wall);
		void insertVertical(VerticalWallSegment* wall);
	};
}
#endif