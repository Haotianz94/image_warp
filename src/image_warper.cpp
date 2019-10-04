// #include "CV_Util2.h"
// #include <direct.h>
#include <fstream>
#include "image_warper.h"

using namespace cv;


void loadCtrPnts()
{
	std::vector<cv::Point2f> ctrPntSrc, ctrPntDst;

	ctrPntSrc.push_back(Point2f(100, 100));
	ctrPntSrc.push_back(Point2f(100, 400));
	ctrPntSrc.push_back(Point2f(400, 100));
	ctrPntSrc.push_back(Point2f(400, 400));

	ctrPntDst.push_back(Point2f(50, 50));
	ctrPntDst.push_back(Point2f(50, 450));
	ctrPntDst.push_back(Point2f(450, 50));
	ctrPntDst.push_back(Point2f(450, 450));

	return; 
}

// void ImageWarper::medianFilter(Mat& image, int filter)
// {
// 	for(int x=0; x<image.cols; x++)
// 		for(int y=0; y<image.rows; y++)
// 		{
// 			uchar ptr = image.at<uchar>(y,x);
// 			if(ptr > filter)//10
// 				image.at<uchar>(y,x) = 255;
// 			else
// 				image.at<uchar>(y,x) = 0;
// 		}

// 		int h = image.rows;
// 		int w = image.cols;
// 		for(int x=0; x<image.cols; x++)
// 			for(int y=0; y<image.rows; y++)
// 			{
// 				int num = 0;
// 				int sum = 0;
// 				for(int i=-1; i<2; i++)
// 					for(int j=-1; j<2; j++)
// 						if(x+i>=0 && x+i<w && y+j>=0 && y+j<h)
// 						{
// 							uchar ptr = image.at<uchar>(y+j, x+i);
// 							sum ++;
// 							if(ptr == 0)
// 								num ++;
// 						}
// 				if(num > 4 || sum < 9)
// 				{
// 					image.at<uchar>(y, x) = 0; 
// 				}
// 				else
// 					image.at<uchar>(y, x) = 255;
// 			}
// }