#include <iostream>
#include "StrategyClassify.h"

using namespace amee;

StrategyClassify::StrategyClassify(ros::Publisher& pub) {
	mPub = pub;
	mRunning = false;
}

StrategyClassify::~StrategyClassify() {
}

void StrategyClassify::init(const StrategyData& data) {
	mRunning = true;
}

bool StrategyClassify::isRunning() const {
	return mRunning;
}

void StrategyClassify::doControl(const StrategyData& data) {
}

