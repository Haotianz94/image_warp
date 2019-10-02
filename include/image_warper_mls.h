#ifndef IMAGE_WARPER_MLS_H_
#define IMAGE_WARPER_MLS_H_

#include <vector>
#include <math.h>
#include "image_warper.h"

#define BOUNDED(x, y, W, H) ( (x>0) && (x<W) && (y>0) && (y<H))


struct matrix2 
{
	double c11,c12,c21,c22;
};


class ImageWarperMLS: public ImageWarper  
{
private:
	// std::vector<cv::CVPoint2d32f> _ctrPntSrc32f, _ctrPntDst;
	// void _convertCtrPnt();
	CvPoint2D32f _warpPnt(CvPoint2D32f point);
	cv::Vec3b _bilinerInterpolate(cv::Point2f pos, cv::Mat& img);

public:
	ImageWarperMLS();
	~ImageWarperMLS();
	cv::Mat warp(cv::Mat& imgIn);
	void test();

};

#endif