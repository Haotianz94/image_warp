#ifndef IMAGE_WARPER_ASAP_H_
#define IMAGE_WARPER_ASAP_H_

#include "mesh.h"
#include "image_warper.h"

class ImageWarperASAP: public ImageWarper{
private:
	int _gridX, _gridY;
	Mesh _mesh;

	bool _ok(cv::Mat &mask, int x, int y);
	void _generateWarpMesh(Mesh & mesh);

public:
	ImageWarperASAP();
	~ImageWarperASAP();
	cv::Mat ImageWarperASAP::warp(cv::Mat& imgIn, bool bShowGrid /*= 0*/, bool bBiggerBord /*= 0*/ );

};

#endif