#include "SlamModule.h"

int main() {
	std::string calibFile, source, ply;
	SlamModule slamModule;
	slamModule.main(calibFile, source, ply);
	return 0;
}
