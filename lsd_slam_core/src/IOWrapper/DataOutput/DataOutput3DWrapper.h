/*
 * DATAOutput3DWrapper.h
 *
 *  Created on: 17 Oct 2014
 *      Author: Nathan
 */

#ifndef DATAOUTPUT3DWRAPPER_H_
#define DATAOUTPUT3DWRAPPER_H_

#include "../DataOutput/Keyframe.h"
#include "IOWrapper/Output3DWrapper.h"
#include "GUI.h"

namespace lsd_slam {

class Frame;
class KeyFrameGraph;

class DataOutput3DWrapper: public Output3DWrapper {
public:
	DataOutput3DWrapper(int width, int height, GUI & gui);
	virtual ~DataOutput3DWrapper();

	virtual void publishKeyframeGraph(KeyFrameGraph* graph);

	// publishes a keyframe. if that frame already existis, it is overwritten, otherwise it is added.
	virtual void publishKeyframe(Frame* f);

	virtual void updateImage(unsigned char * data);

	// published a tracked frame that did not become a keyframe (i.e. has no depth data)
	virtual void publishTrackedFrame(Frame* f);

	// publishes graph and all constraints, as well as updated KF poses.
	virtual void publishTrajectory(
			std::vector<Eigen::Matrix<float, 3, 1>> trajectory,
			std::string identifier);

	virtual void publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt,
			std::string identifier);

	virtual void publishDebugInfo(Eigen::Matrix<float, 20, 1> data);

	int publishLvl;

private:
	int width, height;
	GUI & gui;
};
}

#endif /* DATAOUTPUT3DWRAPPER_H_ */
