#ifndef _LOGGER
#define _LOGGER

#pragma once

#include<stdio.h>
#include <stdarg.h>

class Logger {
public:
	void startLogging(const char* filename, bool consoleLog = false) {
		fLog = fopen(filename,"wt");
		doConsoleLog = consoleLog;
	}
	void stopLogging() {
		if (NULL != fLog) {
			fclose(fLog);
		}
		fLog = NULL;
		doConsoleLog = false;
	}
	void setConsoleLog(bool l = true) {
		doConsoleLog = l;
	}
	~Logger() {
	  stopLogging();
	}
	void printf(const char* fmt, ...) {
		if (NULL != fLog) {
			va_list args;
			va_start(args, fmt);
			vfprintf(fLog, fmt, args);
			va_end(args);
			fflush(fLog);
		}
		if (doConsoleLog) {
			va_list args;
			va_start(args, fmt);
			vfprintf(stderr, fmt, args);
			va_end(args);
		}
	}
	bool doConsoleLog = false;
	FILE* fLog = NULL;
};

extern Logger logger;

#endif
