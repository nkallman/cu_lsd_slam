#include "LiveSLAMWrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/Parse.h"
#include "util/globalFuncs.h"
#include "util/ThreadMutexObject.h"
//#include "IOWrapper/Pangolin/PangolinOutput3DWrapper.h"
#include "IOWrapper/DataOutput/DataOutput3DWrapper.h"
#include "SlamSystem.h"
#include "util/ImageSource.h"

#include <sstream>
#include <fstream>
#include <dirent.h>
#include <algorithm>

#include "util/Undistorter.h"
#include "util/RawLogReader.h"

#include "opencv2/opencv.hpp"

#include "GUI.h"

//ThreadMutexObject<bool> SlamModule::lsdDone(false);
//RawLogReader * SlamModule::logReader = 0;
//int SlamModule::numFrames = 0;
//ImageSource* SlamModule::imageSource;
//std::vector<std::string> SlamModule::files;
//int SlamModule::w, SlamModule::h, SlamModule::w_inp, SlamModule::h_inp;
//GUI SlamModule::gui;

class SlamModule {
	// TREVOR: IMAGE SOURCE VARIABLE
	private:
		static ThreadMutexObject<bool> lsdDone;
		static RawLogReader * logReader;
		static int numFrames;

		static ImageSource *imageSource;

		static std::vector<std::string> files;
		static int w, h, w_inp, h_inp;

		static GUI gui;

	public:
		static std::string &ltrim(std::string &s);
		static std::string &rtrim(std::string &s);
		static std::string &trim(std::string &s);

		/**
		 * Get directory for images
		 */
		static int getdir(std::string dir, std::vector<std::string> &files);

		/**
		 * Get file a file from a source and place it in a file vector.
		 */
		static int getFile(std::string source, std::vector<std::string> &files);

		/**
		 * Run SLAM inside main process.
		 */
		static void run(lsd_slam::SlamSystem * system, lsd_slam::Undistorter* undistorter,
				lsd_slam::Output3DWrapper* outputWrapper, Sophus::Matrix3f K);

		/**
		 * Execute the main SLAM process.
		 */
		static int main(std::string calibFile, std::string source, std::string ply, bool save);
};
