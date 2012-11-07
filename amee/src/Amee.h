#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string>
#include <unistd.h>
#include <iostream>
#include <stdarg.h>

#ifndef AMEE_H
#define AMEE_H

#define WHEEL_BASE 0.237f

// Can only handle C types
void log(char* fmt, ...);

#endif
