
#include "Amee.h"
#include <sys/time.h>

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
inline double timeNow(){
	struct timeval time;
	gettimeofday(&time, NULL);
	
	return time.tv_sec+double(time.tv_usec)/1000000.0;
}