#ifndef IMAGE_WARPER_ASAP_H_
#define IMAGE_WARPER_ASAP_H_

#include "mesh.h"
#include "image_warper.h"

// class ImageWarperASAP: public ImageWarper{
// private:
// 	int _gridX, _gridY;

// public:
// 	ImageWarperASAP();
// 	~ImageWarperASAP();
// 	cv::Mat ImageWarperASAP::warp(cv::Mat& imgIn, bool bShowGrid /*= 0*/, bool bBiggerBord /*= 0*/ );

// };

cv::Mat warpASAPInterface(cv::Mat, cv::Mat, cv::Mat);

cv::Mat warpASAP(cv::Mat& imgIn, int gridX, int gridY);

void testASAP();

#endif