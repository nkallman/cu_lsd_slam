/*
 * ImageSource.cpp
 *
 *  Created on: Feb 23, 2016
 *      Author: scoutdroneuser
 */

#include "ImageSource.h"

// ImageSource Interface Methods

ImageSource::ImageSource() {
	w = 0;
	h = 0;
	index = 0;
}

//bool ImageSource::IsAtEnd() {
//	return false;
//}
//
//cv::Mat ImageSource::GetNextImage() {
//	cv::Mat r;
//	return r;
//}

void ImageSource::SetWidthAndHeight(int width, int height) {
	w = width;
	h = height;
}

// CameraImageSource Implementation

CameraImageSource::CameraImageSource(int cameraIndex) :
		ImageSource() {
	camera = cv::VideoCapture(cameraIndex);

	if (!camera.isOpened()) {
		printf("Error: Could not find camera at index %d", cameraIndex);
	}
}

bool CameraImageSource::IsAtEnd() {
	return false;
}

cv::Mat CameraImageSource::GetNextImage() {
	cv::Mat imageDist;
	cv::Mat initImage;

	camera >> initImage;

	cv::cvtColor(initImage, imageDist, CV_RGB2GRAY);

	if (imageDist.rows != h || imageDist.cols != w) {
		if (imageDist.rows * imageDist.cols == 0) {
			printf("failed to take image.\n");
		} else {
			printf(
					"image has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
					w, h, imageDist.cols, imageDist.rows);
		}
	}

	index++;

	return imageDist;
}

// LogReaderImageSource Implementation

LogReaderImageSource::LogReaderImageSource(RawLogReader *lr) :
		ImageSource() {
	logReader = lr;
}

bool LogReaderImageSource::IsAtEnd() {
	return index == logReader->getNumFrames() - 1;
}

cv::Mat LogReaderImageSource::GetNextImage() {
	cv::Mat imageDist;

	if (IsAtEnd()) {
		return imageDist;
	}

	logReader->getNext();

	cv::Mat3b img(h, w, (cv::Vec3b *) logReader->rgb);

	cv::cvtColor(img, imageDist, CV_RGB2GRAY);

	index++;

	return img;
}

// FileListImageSource Implementation

FileListImageSource::FileListImageSource(std::vector<std::string> fileList) :
		ImageSource() {
	files = fileList;
	index = 0;
}

bool FileListImageSource::IsAtEnd() {
	return index == files.size() - 1;
}

cv::Mat FileListImageSource::GetNextImage() {
	cv::Mat imageDist;

	if (IsAtEnd()) {
		return imageDist;
	}

	imageDist = cv::imread(files[index], CV_LOAD_IMAGE_GRAYSCALE);

	if (imageDist.rows != h || imageDist.cols != w) {
		if (imageDist.rows * imageDist.cols == 0) {
			printf("failed to load image %s! skipping.\n",
					files[index].c_str());
		} else {
			printf(
					"image %s has wrong dimensions - expecting %d x %d, found %d x %d. Skipping.\n",
					files[index].c_str(), w, h, imageDist.cols, imageDist.rows);
		}
	}
}

// CameraModuleImageSource Implementation
CameraModuleImageSource::CameraModuleImageSource() :
		ImageSource() {

}

bool CameraModuleImageSource::IsAtEnd() {
	return imageQueue == null;
}

// Reads in next set of images and returns the left image from this new set.
cv::Mat CameraModuleImageSource::GetNextImage() {
	left = getGrayImage();

	if(isStereo) {
		right = getGrayImage();
	}

	return left;
}

// Gets the right image from last read. Does not read new set of images.
cv::Mat CameraModuleImageSource::GetRight() {
	return right;
}

// Gets the left image from the last read. Does not read new images.
cv::Mat CameraModuleImageSource::GetLeft() {
	return left;
}

bool CameraModuleImageSource::SetStereo(bool isStereoCam) {
	isStereo = isStereoCam;
}

cv::Mat CameraModuleImageSource::getGrayImage() {
	cv::Mat imageDist;

	int i = 0;
	while(!imageQueue.try_pop(imageDist) && i < 1000) {
		i++;
	}

	cv::cvtColor(imageDist, imageDist, CV_RGB2GRAY);
	return imageDist;
}
