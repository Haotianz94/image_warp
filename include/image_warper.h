#ifndef _IMAGE_WARPER_H_
#define _IMAGE_WARPER_H_

#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>


class ImageWarper
{
protected:
	int _frameW, _frameH;
	std::vector<cv::Point2f> _ctrPntSrc, _ctrPntDst;
	// void medianFilter(cv::Mat& image, int filter);

public:
	ImageWarper();
	~ImageWarper();
	cv::Mat loadImage(std::string);
	void loadCtrPnts();
	
	virtual void test() = 0;
};

#endif