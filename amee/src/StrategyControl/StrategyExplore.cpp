#include <iostream>
#include "StrategyExplore.h"

using namespace amee;

StrategyExplore::StrategyExplore(ros::Publisher& pub) {
	mPub = pub;
	mRunning = false;
}

StrategyExplore::~StrategyExplore() {
}

void StrategyExplore::init(const StrategyData& data) {
	mRunning = true;
}

bool StrategyExplore::isRunning() const {
	return mRunning;
}

void StrategyExplore::doControl(const StrategyData& data) {
}

