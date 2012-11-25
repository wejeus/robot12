#include "PhaseTwoControl.h"

using namespace amee;
using namespace std;

PhaseTwoControl::PhaseTwoControl(ros::NodeHandle &nodeHandle)
	: mNextTargetNode(-1)
	, mCurrentNode(-1)
	, mIsFinishing(false)
	, mTargetReached(true) // To force amee to search for next target from starting position.
{
	mGraph = new Graph();

	// ifstream infile;

 //    infile.open("maze.map");
 //    string line;

 //    // Read meta data
 //    getline(infile, line);
 //    int numNodes = 3;
 //   	int numEdges = 2;
 //   	ó
 //    cout << "META: " << line << endl;

 //    if (!infile.is_open()) {
 //    	cout << "Cant open map file!" << endl;
 //    	exit(-1);
 //    }

 //    // Read nodes
 //    for (int i = 0; i < numNodes; ++i) {
	// 	getline(infile, line);
	// 	cout << "NODE: " << line << endl;
 //    }
    
 //    // Read edges
 //    for (int i = 0; i < numEdges; ++i) {
	// 	getline(infile, line);
	// 	cout << "EDGE: " << line << endl;
 //    }

 //    infile.close();

	// float correct_dists[] = {0.0f, 7.0f, 10.3078f, 17.0346f, 20.6218f, 13.3417f};
	// int correct_path[]  = {0, 0, 0, 2, 5, 0};

	float x_list[] = {0.0f, 7.0f,  2.5f,  7.0f,  5.0f,  3.0f};
	float y_list[] = {0.0f, 0.0f, 10.0f, 15.0f, 20.0f, 13.0f};
	float theta_list[] = {0.0f, 7.0f,  2.5f,  7.0f,  5.0f,  3.0f};
	int type_list[] = {1, 0, 3, 2, 1, 3};
	
	for(size_t i=0; i<6; ++i){
		Pose pose;
		pose.x = x_list[i];
		pose.y = y_list[i];
		pose.theta = theta_list[i];
		mGraph->addNode(pose, type_list[i]);
	}

	//adding edges
	mGraph->addEdges(0,1);
	mGraph->addEdges(0,2);
	mGraph->addEdges(0,5);

	mGraph->addEdges(1,2);
	mGraph->addEdges(1,3);

	mGraph->addEdges(2,3);
	mGraph->addEdges(2,5);

	mGraph->addEdges(3,4);

	mGraph->addEdges(4,5);

	
	// mPhaseInfo = nodeHandle.subscribe("/StrategyControl/PhaseInfo", 10, &amee::PhaseTwoControl::phaseInfoCallback);
	// mStrategyControl = nodeHandle.advertise<amee::StrategyCommand>("/StrategyControl/StrategyCommand", 100);
	
	vector<NodeMsg*> nodes = mGraph->getNodes();

	for (vector<NodeMsg*>::iterator it = nodes.begin(); it < nodes.end(); ++it) {
		cout << "Test: " << (*it)->nodeID << endl;
		if ((*it)->type == Graph::NODE_TAG) {
			cout << "Node: " << (*it)->nodeID << " is a TAG!" << endl;
			mTarget.push_back((*it)->nodeID)
		}
	}
	mCurrentNode = mTargets.front();
	mIsRunning = true;
}

PhaseTwoControl::~PhaseTwoControl() {
	delete mGraph;
}

// Maybe better to return every node reached?
void PhaseTwoControl::phaseInfoCallback(const std_msgs::Int32 &msg) {
	if (msg.data == 1) {
		cout << "REACHED TARGET!" << endl;
		mTargetReached = true;
		mCurrentNode = mNextTargetNode;
	}	
}

void PhaseTwoControl::rescue() {
// Graph::NODE_ROTATE_LEFT
// amee/Pose pose
// uint32[] edges
// uint32 nodeID
// uint32 type
// TYPE_STRATEGY_GET_OUT = 3;
// TYPE_STRATEGY_GO_TO = 4;

	if (mTargetReached) {
		// Determine next target
		mNextTargetNode = mTargets.back();
		mTargets.pop_back();
		// calculate time needed to travel there and back to maze exit

		// if within time bound: 
			// StrategyGoTo.publish(currentPosition, nextTarget)
			StrategyCommand command;
			command.type = StrategyControl::TYPE_STRATEGY_GO_TO;
			command.x = mCurrentNode;
			command.y = mNextTargetNode;
			mStrategyControl.publish(command);
		// else 
			// find path to exit, go there and finish phase.
			// mIsFinishing = true;
			// StrategyCommand command;
			// command.type = StrategyControl::TYPE_STRATEGY_GET_OUT;
			// mStrategyControl.publish(command);

		mTargetReached = false;
	} else if (mIsFinishing && mTargetReached) {
		cout << "Phase 2 finished." << endl;
		mIsRunning = false;
	}
}