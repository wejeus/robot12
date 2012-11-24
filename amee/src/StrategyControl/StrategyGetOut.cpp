#include <iostream>
#include "StrategyGetOut.h"

using namespace amee;

StrategyGetOut::StrategyGetOut(ros::Publisher& pub) {
	mPub = pub;
	mRunning = false;
}

StrategyGetOut::~StrategyGetOut() {
}

void StrategyGetOut::init(const StrategyData& data) {
	mRunning = true;
}

bool StrategyGetOut::isRunning() const {
	return mRunning;
}

void StrategyGetOut::doControl(const StrategyData& data) {
}

