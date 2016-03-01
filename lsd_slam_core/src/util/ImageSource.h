/*
 * ImageSource.h
 *
 *  Created on: Feb 23, 2016
 *      Author: scoutdroneuser
 */

#ifndef UTILITIES_IMAGESOURCE_H_
#define UTILITIES_IMAGESOURCE_H_

#include <string>
#include "RawLogReader.h"
#include "../../../../lsdslamd/src/lsdslamd/h/CameraModule.h"
#include <tbb/concurrent_queue.h>

class ImageSource {
public:
	~ImageSource() {
	}
	;
	ImageSource();
	virtual bool IsAtEnd() = 0;
	virtual cv::Mat GetNextImage() = 0;
	void SetWidthAndHeight(int width, int height);

protected:
	int h;
	int w;
	unsigned int index;
};

class LogReaderImageSource: public ImageSource {
public:
	LogReaderImageSource(RawLogReader *lr);
	bool IsAtEnd();
	cv::Mat GetNextImage();

private:
	RawLogReader *logReader;
};

class FileListImageSource: public ImageSource {
public:
	FileListImageSource(std::vector<std::string> fileList);
	bool IsAtEnd();
	cv::Mat GetNextImage();

private:
	std::vector<std::string> files;
};

class CameraImageSource: public ImageSource {
public:
	CameraImageSource(int cameraIndex);
	bool IsAtEnd();
	cv::Mat GetNextImage();

private:
	cv::VideoCapture camera;
};

// This class will use concurrent_bounded_queue from CameraModule.h as the source of its images
class CameraModuleImageSource: public ImageSource {
public:
	CameraModuleImageSource();
	bool IsAtEnd();

	// Reads a new image or set of images and returns the left image
	cv::Mat GetNextImage();
	// Returns the left image without reading in new images
	cv::Mat GetLeftImage();
	// Returns the right image without reading in new images
	cv::Mat GetRightImage();

	// Set the ImageSource to read left and right images
	void SetStereo(bool isStereo);

private:
	cv::Mat getGrayImage();
	cv::Mat left;
	cv::Mat right;
	bool isStereo;
};

#endif /* UTILITIES_IMAGESOURCE_H_ */
