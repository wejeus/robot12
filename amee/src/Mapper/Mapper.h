#ifndef MAPPER_H
#define MAPPER_H

#include "ros/ros.h"
#include "amee/IRDistances.h"
#include "amee/FollowWallStates.h"
#include "amee/Tag.h"
#include "Map.h"
#include <vector>
#include "amee/MapVisualization.h"

namespace amee {

class Mapper {

	public:
		Mapper(ros::Publisher pub);
		~Mapper();
		void receive_distances(const amee::IRDistances::ConstPtr &msg);
		void receive_pose(const amee::Pose::ConstPtr &msg);
		void receive_tag(const amee::Tag::ConstPtr &msg);
		void receive_FollowWallState(const amee::FollowWallStates::ConstPtr &msg);
		void doMapping();
		void init(const amee::FollowWallStates::ConstPtr &msg);
		void setVisualizationPublisher(ros::Publisher pub);

		enum MappingState {Pause, NextToWall, Rotating};
	
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

		MappingState mMappingState;

		amee::Map::Point mCurrentPos; // current position in map coordinates
		float mCurrentAngle; // current angle in map coordinates

		struct Measurement {
			bool valid;
			amee::Map::Point pos;
		};

		static const int 

		std::vector<Measurement> mMeasurements;
		std::vector<Map::Point> mTagPositions;

		MapVisualization mVis;

		void calculateMeasurements();
		bool isValidDistance(float dist);
		void visualize();
		void cleanMap();
	};
}
#endif