#include <omp.h>
#include "image_warper_mls.h"

using namespace cv;
using namespace std;


vector<Point2f> ctrPntSrc, ctrPntDst;

Mat warpMLS(Mat& imgIn)
{
	omp_set_num_threads(16);
	
	int frameW = imgIn.cols;
	int frameH = imgIn.rows;

	Mat imgOut(frameH, frameW, CV_8UC3, Scalar(0, 0, 0));
	#pragma omp parallel for
	for(int y = 0; y <= frameH-1; y ++)
		for(int x = 0; x <= frameW-1; x ++)
		{
			// if(x == frameW - 1)
			// 	std::cout << y << std::endl;

			CvPoint2D32f q = warpPnt(cv::Point(x, y));

			if(!BOUNDED(q.x, q.y, frameW, frameH))
				continue;
			imgOut.at<Vec3b>(y, x) = bilinerInterpolate(q, imgIn);
		}
	return imgOut;
}


Vec3b bilinerInterpolate(Point2f pos, Mat& img)
{
	float X = pos.x;
	float Y = pos.y;
	
	if(X < 0 || X >= img.cols-1 || Y < 0 || Y >= img.rows-1)
		return Vec3b(0, 0, 0);

	Vec3b p,q,r;
	p = img.at<Vec3b>((int)Y, (int)X) + (X - (int)X) * (img.at<Vec3b>((int)Y ,(int)X + 1) - img.at<Vec3b>((int)Y, (int)X));
	q = img.at<Vec3b>((int)Y + 1,(int)X) + (X - (int)X) * (img.at<Vec3b>((int)Y + 1, (int)X + 1) - img.at<Vec3b>((int)Y + 1,(int)X));
	r = p + (Y - (int)Y) * (q - p);
	return r;
}


CvPoint2D32f warpPnt(CvPoint2D32f point)
{
		int nSize=ctrPntDst.size();
		//compute weight
		std::vector<double> Weight;
		for (int i1=0;i1<nSize;i1++)
		{
			double dtemp=( pow(	static_cast<double>(point.x-ctrPntDst[i1].x),  2 )+
										   pow(	static_cast<double>(point.y-ctrPntDst[i1].y),	2)		);

			if(dtemp==0.0)
			{
				dtemp=dtemp+0.0000001;
			}
			
			double temp=1/dtemp;

			Weight.push_back(temp);
		}
		//compute pStar,QStar
		double dSumW=0;
		double dSumPx=0,dSumPy=0;
		double dSumQx=0,dSumQy=0;
		for (int i2=0;i2<nSize;i2++)
		{
			dSumW=dSumW+Weight[i2];
			dSumPx=dSumPx+Weight[i2]*(double)ctrPntDst[i2].x;
			dSumPy=dSumPy+Weight[i2]*(double)ctrPntDst[i2].y;
			dSumQx=dSumQx+Weight[i2]*(double)ctrPntSrc[i2].x;
			dSumQy=dSumQy+Weight[i2]*(double)ctrPntSrc[i2].y;
		}
		CvPoint2D32f PStar,QStar;
		PStar.x=(dSumPx/dSumW);
		PStar.y=(dSumPy/dSumW);
		QStar.x=(dSumQx/dSumW);
		QStar.y=(dSumQy/dSumW);

		std::vector<CvPoint2D32f> PHat,QHat;
		for (int i3=0;i3<nSize;i3++)
		{
			CvPoint2D32f tempP;
			tempP.x=ctrPntDst[i3].x-PStar.x;
			tempP.y=ctrPntDst[i3].y-PStar.y;
			PHat.push_back(tempP);
			CvPoint2D32f tempQ;
			tempQ.x=ctrPntSrc[i3].x-QStar.x;
			tempQ.y=ctrPntSrc[i3].y-QStar.y;
			QHat.push_back(tempQ);
		}
		//compute the inverse Matrix
		matrix2 Mat1={0,0,0,0};
		for (int i4=0;i4<nSize;i4++)
		{
			Mat1.c11=Mat1.c11+Weight[i4]* pow(	static_cast<double>(PHat[i4].x), 2 );
			Mat1.c12=Mat1.c12+Weight[i4]*(double)(PHat[i4].x*PHat[i4].y);
			Mat1.c21=Mat1.c12;
			Mat1.c22=Mat1.c22+Weight[i4]* pow( static_cast<double>(PHat[i4].y), 2 );
		}

		double tempM=Mat1.c11*Mat1.c22-Mat1.c12*Mat1.c21;
		matrix2 MatInver;
		MatInver.c11=Mat1.c22/tempM;
		MatInver.c12=-Mat1.c12/tempM;
		MatInver.c21=-Mat1.c21/tempM;
		MatInver.c22=Mat1.c11/tempM;

		//compute the scale Aj
		std::vector<double> dScale;
		for (int i5=0;i5<nSize;i5++)
		{
			double tempS;
			tempS=((double)(point.x-PStar.x)*MatInver.c11+(double)(point.y-PStar.y)*MatInver.c21)*Weight[i5]*(double)PHat[i5].x
				+((double)(point.x-PStar.x)*MatInver.c12+(double)(point.y-PStar.y)*MatInver.c22)*Weight[i5]*(double)PHat[i5].y;
			dScale.push_back(tempS);
		}
		//compute the New Point
		CvPoint2D32f NewPoint=cvPoint2D32f(0,0);
		for (int i6=0; i6<nSize; i6++)
		{
			NewPoint.x=NewPoint.x+(dScale[i6]*QHat[i6].x);
			NewPoint.y=NewPoint.y+(dScale[i6]*QHat[i6].y);
		}
		NewPoint.x=NewPoint.x+QStar.x;
		NewPoint.y=NewPoint.y+QStar.y;
		return NewPoint;	
}


void testMLS()
{
	
	Mat imgIn = imread("./test.jpg");
	std::cout << "Load image..." << std::endl;

	ctrPntSrc.push_back(Point2f(100, 100));
	ctrPntSrc.push_back(Point2f(100, 400));
	ctrPntSrc.push_back(Point2f(400, 100));
	ctrPntSrc.push_back(Point2f(400, 400));

	ctrPntDst.push_back(Point2f(50, 50));
	ctrPntDst.push_back(Point2f(50, 450));
	ctrPntDst.push_back(Point2f(450, 50));
	ctrPntDst.push_back(Point2f(450, 450));

	Mat imgOut = warpMLS(imgIn);
	imwrite("./warp_MLS.jpg", imgOut);	
}


Mat warpMLSInterface(Mat imgIn, Mat ctrPntSrcMat, Mat ctrPntDstMat)
{
	assert("Control point sets size does not match!" && ctrPntSrcMat.rows == ctrPntDstMat.rows);
	ctrPntSrc.clear();
	ctrPntDst.clear();

	for(int i = 0; i < ctrPntSrcMat.rows; i ++)
	{	
		ctrPntSrc.push_back(Point2f(ctrPntSrcMat.at<double>(i, 0), ctrPntSrcMat.at<double>(i, 1)));
		ctrPntDst.push_back(Point2f(ctrPntDstMat.at<double>(i, 0), ctrPntDstMat.at<double>(i, 1)));
	}

	return warpMLS(imgIn);
}


double testAdd(double a, double b){
	return a + b;
}


cv::Mat testIO(cv::Mat a, cv::Mat b)
{
	cout << a.cols << " " << a.rows << endl;
	cout << b.cols << " " << b.rows << endl;
	return a.clone();
}
