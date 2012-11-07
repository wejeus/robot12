
#include "Amee.h"

// Can only handle C types
void log(char* fmt, ...) {
    va_list args;
    va_start(args, fmt);

    #ifdef ROS
    // ROS_DEBUG(fmt, args);
    vprintf(fmt, args);
    #else
    vprintf(fmt, args);
    #endif

    va_end(args);
}