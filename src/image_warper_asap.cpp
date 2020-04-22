#include "image_warper_asap.h"
#include "asap_solver.h"

// #include "CV_Util.h" //???
using namespace cv;
using namespace std;


vector<Point2f> ctrPntSrc, ctrPntDst;

Mat warpASAPInterface(Mat imgIn, Mat ctrPntSrcMat, Mat ctrPntDstMat)
{
	assert("Control point sets size does not match!" && ctrPntSrcMat.rows == ctrPntDstMat.rows);
	ctrPntSrc.clear();
	ctrPntDst.clear();

	for(int i = 0; i < ctrPntSrcMat.rows; i ++)
	{	
		ctrPntSrc.push_back(Point2f(ctrPntSrcMat.at<double>(i, 0), ctrPntSrcMat.at<double>(i, 1)));
		ctrPntDst.push_back(Point2f(ctrPntDstMat.at<double>(i, 0), ctrPntDstMat.at<double>(i, 1)));
	}

	return warpASAP(imgIn);
}


Mat warpASAP(Mat& imgIn, int gridX, int gridY)
{	
	int frameW = imgIn.cols;
	int frameH = imgIn.rows;

	Mat_<Vec2f> deformedMesh;
	Mat imgOut(frameH, frameW, CV_8UC3, Scalar(0, 0, 0));
	
	ASAPSolver asap(frameW, frameH, gridX, gridY);
	asap.solve(ctrPntSrc, ctrPntDst, deformedMesh);

	//draw new Mesh
	
	Mat deformedMeshCanvas(frameH, frameW, CV_8UC3, Scalar(255, 255, 255));
	for(int y = 0; y < deformedMesh.rows; y++)
		for(int x = 0; x < deformedMesh.cols; x++)
		{
			Vec2f pos = deformedMesh.at<Vec2f>(y, x);
			circle(deformedMeshCanvas, Point(pos[0], pos[1]), 2, Scalar(0, 0, 0), -1);
		}
	imshow("deformedMesh", deformedMeshCanvas);
	waitKey(0);
	
	asap.warpBruteForce(imgIn, deformedMesh, imgOut, true, false);

	return imgOut;
}


void testASAP()
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

	Mat imgOut = warpASAP(imgIn, 10, 10);
	imwrite("./warp_ASAP.jpg", imgOut);	
}