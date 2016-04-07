#include "SlamModule.h"
#include <string.h>

int main() {
	std::string calibFile, source, ply;
	SlamModule slamModule;
	SlamModule::main(calibFile, source, ply);
	return 0;
}
