#include "lib.h"

Logger logger;

void TestLogger::test(const char* message) {
	logger.printf(message);
} 