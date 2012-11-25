#include "ros/ros.h"

#include "StrategyControl.h"
#include "StrategyExplore.h"
#include "StrategyClassify.h"
#include "StrategyGoTo.h"

#include <iostream>
#include <cmath>

#include "amee/StrategyCommand.h"
#include "amee/MovementCommand.h"
#include "amee/NodeMsg.h"

#include <std_msgs/Int32.h>