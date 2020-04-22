#ifndef ASAP_SOLVER_H
#define ASAP_SOLVER_H

#include "mesh.h"

#include "ceres/ceres.h"

using ceres::NumericDiffCostFunction;
using ceres::CENTRAL;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

///////////////////////////////////
//Data Term of ASAP
// E_d(V^)=sum_p||V_p^w_p-p^||^2
// ////////////////////////////////
// 
struct CostDataTermXY{
public:
	CostDataTermXY(std::vector<double> & weights, cv::Point2f control_pt);

	template <typename T> bool operator()(const T* const x00,
		const T* const x10,
		const T* const x01,
		const T* const x11,
		const T* const y00,
		const T* const y10,
		const T* const y01,
		const T* const y11,
		T* residual) const{


			T _x = T(x00[0]*T(weight00_)+x10[0]*T(weight10_)+x01[0]*T(weight01_)+x11[0]*T(weight11_)) - T(x_);
			T _y = T(y00[0]*T(weight00_)+y10[0]*T(weight10_)+y01[0]*T(weight01_)+y11[0]*T(weight11_)) - T(y_);

			//cout<<_x<<endl;
			//cout<<_y<<endl;

			//residual[0] = sqrt(_x*_x+_y*_y);
			residual[0] = _x;
			residual[1] = _y;
			//cout<<residual[0]<<endl;
			return true;
	}
private:
	double weight00_, weight10_, weight01_, weight11_;
	double x_, y_;
};

struct CostDataTermX{
public:
	CostDataTermX(std::vector<double> & weights, cv::Point2f control_pt);

	template <typename T> bool operator()(const T* const x00,
									  const T* const x10,
									  const T* const x01,
									  const T* const x11,
									  
						
									  T* residual) const{


		T _x = T(x00[0]*T(weight00_)+x10[0]*T(weight10_)+x01[0]*T(weight01_)+x11[0]*T(weight11_)) - T(x_);
		
		
		//cout<<_x<<endl;
		//cout<<_y<<endl;

		//residual[0] = sqrt(_x*_x+_y*_y);
		residual[0] = _x;
		//residual[1] = _y;
		//cout<<residual[0]<<endl;
		return true;
	}
private:
	double weight00_, weight10_, weight01_, weight11_;
	double x_, y_;
};


struct CostDataTermY{
public:
	CostDataTermY(std::vector<double> & weights, cv::Point2f control_pt);

	template <typename T> bool operator()(const T* const y00,
		const T* const y10,
		const T* const y01,
		const T* const y11,
		T* residual) const{


		T _y = T(y00[0]*T(weight00_)+y10[0]*T(weight10_)+y01[0]*T(weight01_)+y11[0]*T(weight11_)) - T(y_);
			//T _y = T(x00[1]*T(m_weights[0])+x10[1]*T(m_weights[1])+x01[1]*T(m_weights[2])+x11[1]*T(m_weights[3])) - T(m_ctr_pt.y);

			//cout<<_x<<endl;
			//cout<<_y<<endl;

			//residual[0] = sqrt(_x*_x+_y*_y);
			residual[0] = _y;
			//residual[1] = _y;
			//cout<<residual[0]<<endl;
			return true;
	}
private:
	double weight00_, weight10_, weight01_, weight11_;
	double x_, y_;
};

//////////////////////////////////////
//Smooth Term
// E_s(V^)=sum_v||v^-v1^-s*R90*(v0^-v1^)||^2
// ////////////////////////////////////
struct CostSmoothTermX{
	CostSmoothTermX(double lambda=1, double ratio = 0.5):m_lambda(lambda), m_ratio(ratio){}
	/*template <typename T> bool operator()(const T* const v,
		                              const T* const v0,
									  const T* const v1,
									  
									  T* residual) const{
		T x1 = v[0]-v1[0];
		T y1 = v[1]-v1[1];
		T x2 = v0[0]-v1[0];
		T y2 = v0[1]-v1[1];

		//T s = len1/len2;
		//cout<<s<<endl;
		T tx1 = -T(m_ratio)*y2;
		T ty1 = T(m_ratio)*x2;
		
		T x = x1 - tx1;
		T y = y1 - ty1;
		residual[0] = T(sqrt(m_lambda))*sqrt(x*x + y*y);
		//residual[1] = T(m_lambda)*y;
		return true;
	}*/
	template <typename T> bool operator()(const T* const xv,
		                              const T* const xv0,
									  const T* const xv1,
									  const T* const yv,
									  const T* const yv0,
									  const T* const yv1,
									  
									  T* residual) const{
		T x1 = xv[0]-xv1[0];
		T y1 = yv[0]-yv1[0];
		T x2 = xv0[0]-xv1[0];
		T y2 = yv0[0]-yv1[0];

		//T s = len1/len2;
		//cout<<s<<endl;
		T tx1 = T(m_ratio)*y2;
		T ty1 = -T(m_ratio)*x2;
		
		T x = x1 - tx1;
		//T y = y1 - ty1;
		residual[0] = T(sqrt(m_lambda))*x;//T(sqrt(m_lambda))*sqrt(x*x + y*y);
		//residual[1] = T(m_lambda)*y;
		return true;
	}
private:
	double m_lambda;
	double m_ratio;

};
struct CostSmoothTermY{
	CostSmoothTermY(double lambda=1, double ratio = 0.5):m_lambda(lambda), m_ratio(ratio){}
	/*template <typename T> bool operator()(const T* const v,
		                              const T* const v0,
									  const T* const v1,
									  
									  T* residual) const{
		T x1 = v[0]-v1[0];
		T y1 = v[1]-v1[1];
		T x2 = v0[0]-v1[0];
		T y2 = v0[1]-v1[1];

		//T s = len1/len2;
		//cout<<s<<endl;
		T tx1 = -T(m_ratio)*y2;
		T ty1 = T(m_ratio)*x2;
		
		T x = x1 - tx1;
		T y = y1 - ty1;
		residual[0] = T(sqrt(m_lambda))*sqrt(x*x + y*y);
		//residual[1] = T(m_lambda)*y;
		return true;
	}*/
	template <typename T> bool operator()(const T* const xv,
		const T* const xv0,
		const T* const xv1,
		const T* const yv,
		const T* const yv0,
		const T* const yv1,

		T* residual) const{
			T x1 = xv[0]-xv1[0];
			T y1 = yv[0]-yv1[0];
			T x2 = xv0[0]-xv1[0];
			T y2 = yv0[0]-yv1[0];

			//T s = len1/len2;
			//cout<<s<<endl;
			T tx1 = T(m_ratio)*y2;
			T ty1 = -T(m_ratio)*x2;

			//T x = x1 - tx1;
			T y = y1 - ty1;
			residual[0] = T(sqrt(m_lambda))*y;//T(sqrt(m_lambda))*sqrt(x*x + y*y);
			//residual[1] = T(m_lambda)*y;
			return true;
	}
private:
	double m_lambda;
	double m_ratio;

};
struct CostSmoothTermXY{
	CostSmoothTermXY(double lambda=1, double ratio = 0.5):m_lambda(lambda), m_ratio(ratio){}

	template <typename T> bool operator()(const T* const xv,
		const T* const xv0,
		const T* const xv1,
		const T* const yv,
		const T* const yv0,
		const T* const yv1,

		T* residual) const{
			T x1 = xv[0]-xv1[0];
			T y1 = yv[0]-yv1[0];
			T x2 = xv0[0]-xv1[0];
			T y2 = yv0[0]-yv1[0];

			//T s = len1/len2;
			//cout<<s<<endl;
			T tx1 = T(m_ratio)*y2;
			T ty1 = -T(m_ratio)*x2;

			T x = x1 - tx1;
			T y = y1 - ty1;
			residual[0] = T(sqrt(m_lambda))*x;//T(sqrt(m_lambda))*sqrt(x*x + y*y);
			residual[1] = T(sqrt(m_lambda))*y;
			return true;
	}
private:
	double m_lambda;
	double m_ratio;

};


class ASAPSolver{
public:
	ASAPSolver(int dimX, int dimY, int gridNumX, int gridNumY);
	ASAPSolver(const Mesh & mesh);
	
	
	void solve(std::vector<cv::Point2f> & pIn, std::vector<cv::Point2f> & pOut, cv::Mat_<cv::Vec2f> &deformed_mesh, double lambda = 1);

	//double solve_adaptive(std::vector<cv::Point2f> & pIn, std::vector<cv::Point2f> & pOut, cv::Mat_<cv::Vec2f> &deformed_mesh);
	// void estimateBundledCameras(cv::Mat_<cv::Vec2f>& deformed_mesh, std::vector<std::vector<cv::Mat>> & bundled_cameras);

	void warpBruteForce(cv::Mat & imgIn, cv::Mat_<cv::Vec2f> & deformed_mesh, cv::Mat & imgOut, bool bShowGrid = 0, bool bigger_bord = 0);

	//void warpViaBundledCameras(cv::Mat & imgIn, std::vector<std::vector<cv::Mat>> & bundled_cameras, cv::Mat & imgOut, bool bShowGrid = 0, bool bigger_bord = 0);
	//void warpViaBundledCameras(cv::Mat &imgIn, BundledCameras bc, cv::Mat & imgOut, bool bShowGrid = 0, bool bigger_bord =0);

private:
	//return 4 surrounding vertices index given a point
	std::vector<cv::Vec2i> gridIdx2verticesIdx(cv::Point2f p);
	Mesh m_mesh;
	//std::vector<std::vector<float>> m_vertices_to_solve;

};

#endif