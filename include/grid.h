#ifndef Grid_H_
#define Grid_H_

// #include "global_header.h"
#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <fstream>

/////
#define FOR(i,a,b) for (int i(a); i < (b); i++)
/////


class Grid{

public:
	Grid(cv::Point2f tl=cv::Point2f(0,0), cv::Point2f dr=cv::Point2f(1,1));
	Grid(float x1, float x2, float y1, float y2);
	Grid (const Grid & rhs);
	Grid & operator = (const Grid & rhs);
	~Grid();
	std::vector<double> getWeights(cv::Point2f v);
	inline cv::Point2f getTL(){return m_tl;}
	inline cv::Point2f getDR(){return m_dr;}
	inline cv::Point2f getTR(){return cv::Point2f(m_dr.x, m_tl.y);}
	inline cv::Point2f getDL(){return cv::Point2f(m_tl.x, m_dr.y);}
	//friend ostream& operator<<(ostream& out, const Grid& q);
	friend std::ostream& operator<<(std::ostream& out, const Grid& q){
		out << "Grid: [" << q.m_tl.x <<", " << q.m_tl.y << "] ["<< q.m_dr.x << ", " << q.m_dr.y<<"]" << std::endl;
		return out;
	}

private:
	cv::Point2f m_tl, m_dr;
	float m_xlen;
	float m_ylen;
};


#endif


