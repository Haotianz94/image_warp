#ifndef MESH_H_
#define MESH_H_

#include "grid.h"


class Mesh{
public:
	Mesh();
	Mesh(float dimX, float dimY, int GridNumX = 16, int GridNumY = 16);
	~Mesh();
	Mesh(const Mesh & rhs);
	Mesh & operator=(const Mesh & rhs);
	//return Mesh Grid
	Grid & getGrid(int xth, int yth);  //0-m_GridNumX-1, 0-m_GridNumY-1
	Grid & getGrid(cv::Point2f v);
	void getGridIdx(cv::Point2f v, int & xth, int &yth);
	inline int getGridNumX(){return m_GridNumX;}
	inline int getGridNumY(){return m_GridNumY;}
	inline int getDIMX(){return m_dimX;}
	inline int getDIMY(){return m_dimY;}
	void cvtVertices2MatVec2f(cv::Mat_<cv::Vec2f>& m);
	void visualize(cv::Mat & image);

	//Grid getGrid1(int x, int y);

	friend std::ostream& operator<<(std::ostream& out, Mesh& q){
		out << "Mesh amount: " << q.m_GridNumX * q.m_GridNumY << std::endl;
		FOR(y, 0, q.m_GridNumY){
			FOR(x, 0, q.m_GridNumX){
				
				out<<"Mesh "<<y*q.m_GridNumX+x<<": "<<q.getGrid(x,y)<<std::endl;
			}
		}
		//out<<"Mesh: ["<<q.m_tl.x<<", "<<q.m_tl.y<<"] ["<<q.m_dr.x<<", "<<q.m_dr.y<<"]"<<endl;
		return out;
	}

private:
	std::vector<Grid> m_vertices;
	float m_GridLenX;
	float m_GridLenY;
	float m_dimX;
	float m_dimY;
	int m_GridNumX;
	int m_GridNumY;
};

#endif