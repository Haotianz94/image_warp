#ifndef IMAGE_WARPER_MLS_H_
#define IMAGE_WARPER_MLS_H_

#include <vector>
#include <math.h>
#include "image_warper.h"
#include <string>

#define BOUNDED(x, y, W, H) ( (x>0) && (x<W) && (y>0) && (y<H))


struct matrix2 
{
	double c11,c12,c21,c22;
};


// class ImageWarperMLS: public ImageWarper  
// {
// private:
// 	// std::vector<cv::CVPoint2d32f> _ctrPntSrc32f, _ctrPntDst;
// 	// void _convertCtrPnt();
// 	CvPoint2D32f _warpPnt(CvPoint2D32f point);
// 	cv::Vec3b _bilinerInterpolate(cv::Point2f pos, cv::Mat& img);

// public:
// 	ImageWarperMLS();
// 	~ImageWarperMLS();
// 	cv::Mat warp(cv::Mat& imgIn);
// 	void test();
// };


CvPoint2D32f warpPnt(CvPoint2D32f, std::vector<cv::Point2f>&, std::vector<cv::Point2f>&);

cv::Vec3b bilinerInterpolate(cv::Point2f, cv::Mat&);

cv::Mat warpMLS(cv::Mat&);

void testMLS();

double testAdd(double, double);

#endif