#ifndef MAPPER_H
#define MAPPER_H

#include "ros/ros.h"
#include "amee/IRDistances.h"
#include "amee/FollowWallStates.h"
#include "amee/Odometry.h"
#include "amee/Tag.h"
#include "Map.h"
#include <vector>
#include "amee/MapVisualization.h"
#include "../Graph/Graph.h"

namespace amee {

class Mapper {

	public:
		Mapper(ros::Publisher pub);
		~Mapper();
		void receive_distances(const amee::IRDistances::ConstPtr &msg);
		void receiveOdometry(const amee::Odometry::ConstPtr &msg);
		void receive_tag(const amee::Tag::ConstPtr &msg);
		void receive_FollowWallState(const amee::FollowWallStates::ConstPtr &msg);
		void doMapping();
		void init();
		void setVisualizationPublisher(ros::Publisher pub);
		void setGraphPublisher(ros::Publisher pub);

		enum MappingState {Pause, NextToWall, Rotating};
	
	private:
		ros::Publisher map_pub;
		ros::Publisher vis_pub;
		ros::Publisher graph_pub;

		amee::IRDistances mDistances;
		amee::Pose mPose;
		amee::Odometry mOdometry;
		amee::Map mMap;
		bool mInitialized;
	
		amee::Graph mGraph;
		int mNodeId;
		int mLastNodeId; // set to -1 if there is no last node

		int mVisualizeTimer;
		int mCleanTimer;

		MappingState mMappingState;

		// amee::Map::Point mCurrentPos; // current position in map coordinates
		// float mCurrentAngle; // current angle in map coordinates

		struct Measurement {
			bool valid;
			amee::Map::Point pos;
		};

		static const int RIGHT_FRONT = 1;
		static const int RIGHT_BACK = 0;
		static const int LEFT_FRONT = 2;
		static const int LEFT_BACK = 3;

		static const float IR_BASE_RIGHT = 0.104;

		std::vector<Measurement> mMeasurements;
		std::vector<Map::Point> mTagPositions;

		MapVisualization mVis;

		void calculateMeasurements();
		bool isValidDistance(float dist);
		void visualize();
		void cleanMap();
		int followedWallDirection();
		void mapping();
		void addNode(int type);
	};
}
#endif