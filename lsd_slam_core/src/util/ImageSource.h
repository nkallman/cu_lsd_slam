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

namespace lsd {

class ImageSource {
public:
	virtual ~ImageSource() {};
	virtual ImageSource();
	virtual bool IsAtEnd() = 0;
	virtual cv::Mat GetNextImage() = 0;
	void SetWidthAndHeight(int width, int height);

private:
	int h;
	int w;
	unsigned int index;
};

class LogReaderImageSource : public ImageSource {
public:
	LogReaderImageSource(const LogReader &lr);
	bool IsAtEnd();
	cv::Mat GetNextImage();

private:
	RawLogReader *logReader;
};

class FileListImageSource : public ImageSource {
public:
	FileListImageSource(std::vector<std::string> fileList);
	bool IsAtEnd();
	cv::Mat GetNextImage();

private:
	std::vector<std::string> files;
};

class CameraImageSource : public ImageSource {
public:
	CameraImageSource(int cameraIndex);
	bool IsAtEnd();
	cv::Mat GetNextImage();

private:
	cv::VideoCapture camera;
};

} /* namespace lsd */

#endif /* UTILITIES_IMAGESOURCE_H_ */
