#ifndef MAPPER_H
#define MAPPER_H

#include "ros/ros.h"
#include "amee/IRDistances.h"
#include "amee/Odometry.h"
#include "Map.h"

namespace amee {

class Mapper {

	public:
		Mapper(ros::Publisher pub);
		~Mapper();
		void receive_distances(const amee::IRDistances::ConstPtr &msg);
		void receive_odometry(const amee::Odometry::ConstPtr &msg);
		void doMapping();
		void init();
		void setVisualizationPublisher(ros::Publisher pub);
	
	private:
		ros::Publisher map_pub;
		ros::Publisher vis_pub;

		amee::IRDistances mDistances;
		amee::Odometry mOdometry;
		amee::Map mMap;
		bool mInitialized;
		amee::Map::Point mStartPos;
		float mStartAngle;

		amee::Map::Point mCurrentPos; // current position in map coordinates
		float mCurrentAngle; // current angle in map coordinates

		struct Measurement {
			bool valid;
			amee::Map::Point pos;
		};

		std::vector<Measurement> mMeasurements;

		void calculateMeasurements();
		bool isValidDistance(float dist);
		void visualize();
	};
}
#endif