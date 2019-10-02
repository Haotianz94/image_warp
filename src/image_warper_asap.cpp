#include "image_warper_asap.h"
#include "asap_solver.h"

// #include "CV_Util.h" //???
using namespace cv;

ImageWarperASAP::ImageWarperASAP()
{

}


ImageWarperASAP::~ImageWarperASAP()
{

}


void ImageWarperASAP::_generateWarpMesh()
{
	ASAPSolver asap(_frameW, _frameH, _gridX, _gridY);
	asap.solve(_ctrPntSrc, _ctrPntDst, this->deformedMesh);
}


bool ImageWarperASAP::_ok(Mat &mask, int x, int y){
	if(x<0||x>=mask.cols||y<0||y>=mask.rows)
		return 0;
	if(mask.at<uchar>(y,x)==255)
		return 1;
	return 0;
}


Mat ImageWarperASAP::warp(Mat& imgIn, 
	bool bShowGrid /*= 0*/, 
	bool bBiggerBord /*= 0*/ )
{
	int xBase = 200;
	int yBase = 200;
	if(!bBiggerBord){
		xBase = 0;
		yBase = 0;
	}

	Mat imgOut(_frameH, _frameW, CV_8UC3, Scalar(0, 0, 0));
	imgOut.create(2*yBase+imgIn.rows, 2*xBase+imgIn.cols,CV_8UC3);
	imgOut.setTo(Vec3b(255,255,255));
	Mat checkMaskOut = Mat::zeros(RC(imgOut),CV_8UC1);
	FOR_PIXELS(y,x,imgIn){
		vector<double> weight = m_mesh.getGrid(Point2f(x,y)).getWeights(Point2f(x,y));
		int xth, yth;
		m_mesh.getGridIdx(Point2f(x,y), xth, yth);
		int newx = xBase+weight[0]*_mesh[yth][xth][0]+weight[1]*_mesh[yth+1][xth][0]\
			+weight[2]*_mesh[yth][xth+1][0]+weight[3]*_mesh[yth+1][xth+1][0];
		
		int newy = yBase+weight[0]*_mesh[yth][xth][1]+weight[1]*_mesh[yth+1][xth][1]\
			+weight[2]*_mesh[yth][xth+1][1]+weight[3]*_mesh[yth+1][xth+1][1];
		/*cout<<"Ori: "<<x<<" "<<y<<endl;
		FOR(j, 0, 4)
			cout<<weight[j]<<" ";
		cout<<endl;
		cout<<newx<<" "<<newy<<endl;*/
		if(newy>=0&& newy<imgOut.rows && newx>=0 && newx<imgOut.cols){
			imgOut.at<Vec3b>(newy, newx) = imgIn.at<Vec3b>(y,x);
			checkMaskOut.at<uchar>(newy, newx) = 255;
		}
	}
	int dir[8][2]={-1,-1, -1,0, -1,1, 0,-1, 0, 1, 1, -1, 1, 0, 1, 1};
	FOR_PIXELS(y,x,imgOut){
		if(checkMaskOut.at<uchar>(y,x) == 0){
			FOR(j, 0, 8){
				int xx = x+dir[j][0];
				int yy = y+dir[j][1];
				if (_ok(checkMaskOut, xx, yy)){
					imgOut.at<Vec3b>(y,x)=imgOut.at<Vec3b>(yy,xx);
					break;
				}
			}
		}
	}
	if(bShowGrid){

		FOR_PIXELS(y,x, _mesh){
			if(x<_mesh.cols-1){
				line(imgOut, Point2f(xBase+_mesh[y][x][0],yBase+_mesh[y][x][1]), Point2f(xBase+_mesh[y][x+1][0],yBase+_mesh[y][x+1][1]), Scalar(255,255,255),3);
			}
			if(y<_mesh.rows-1){
				line(imgOut, Point2f(xBase+_mesh[y][x][0],yBase+_mesh[y][x][1]), Point2f(xBase+_mesh[y+1][x][0],yBase+_mesh[y+1][x][1]), Scalar(255,255,255),3);
			}
			circle(imgOut, Point(xBase+_mesh[y][x][0],yBase+_mesh[y][x][1]),5, Scalar(0,0,255),-1);

		}
		
	}

	return imgOut
}


void test()
{

}
