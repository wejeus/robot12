#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string>
#include <unistd.h>
#include <iostream>
#include <stdarg.h>

#ifndef AMEE_H
#define AMEE_H

#define WHEEL_BASE 0.234f
#define WHEEL_RADIUS 0.0365f
#define TICS_PER_REVOLUTION 225.0f // encoder tics/rev

// Can only handle C types
void log(char* fmt, ...);

inline double timeNow();


#endif
