#include <iostream>
#include "StrategyGo2Tag.h"

using namespace amee;

StrategyGo2Tag::StrategyGo2Tag(ros::Publisher& pub) {
	mPub = pub;
	mRunning = false;
}

StrategyGo2Tag::~StrategyGo2Tag() {
}

void StrategyGo2Tag::init(const StrategyData& data) {
	mRunning = true;
}

bool StrategyGo2Tag::isRunning() const {
	return mRunning;
}

void StrategyGo2Tag::doControl(const StrategyData& data) {
}

