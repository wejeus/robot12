#ifndef MAPPER_H
#define MAPPER_H

#include "ros/ros.h"
#include "amee/IRDistances.h"
#include "amee/FollowWallStates.h"
#include "amee/Tag.h"
#include "Map.h"
#include <vector>

namespace amee {

class Mapper {

	public:
		Mapper(ros::Publisher pub);
		~Mapper();
		void receive_distances(const amee::IRDistances::ConstPtr &msg);
		void receive_pose(const amee::Pose::ConstPtr &msg);
		void receive_tag(const amee::Tag::ConstPtr &msg);
		void doMapping();
		void init(const amee::FollowWallStates::ConstPtr &msg);
		void setVisualizationPublisher(ros::Publisher pub);
	
	private:
		ros::Publisher map_pub;
		ros::Publisher vis_pub;

		amee::IRDistances mDistances;
		amee::Pose mPose;
		amee::Map mMap;
		bool mInitialized;
		amee::Map::Point mStartPos;
		float mStartAngle;

		int mVisualizeTimer;
		int mCleanTimer;

		amee::Map::Point mCurrentPos; // current position in map coordinates
		float mCurrentAngle; // current angle in map coordinates

		struct Measurement {
			bool valid;
			amee::Map::Point pos;
		};

		std::vector<Measurement> mMeasurements;
		std::vector<Map::Point> mTagPositions;

		void calculateMeasurements();
		bool isValidDistance(float dist);
		void visualize();
		void setAngleToN90Deg();
		void cleanMap();
	};
}
#endif