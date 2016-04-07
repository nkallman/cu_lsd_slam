#include "SlamModule.h"

int main() {
	SlamModule slamModule;
	slamModule.main("/home/scoutdroneuser/LSD_room/cameraCalibration.cfg", "/home/scoutdroneuser/LSD_room/images", "result.ply", true);
	return 0;
}
