#ifndef KM
#define KM

#include <Eigen/dense>
#include <vector>
#include "utility.h"

using namespace utility;
using namespace std;

namespace KMSpace{
	
	struct Graph{  //二分图结构体;
		vector<vector<double>> GTable; //邻接矩阵 
		int n;
		int sp;
		int tp;
		vector<int> match;
		vector<double> lx;
		vector<double> ly;
		vector<double> slack;
		vector<bool> visx;
		vector<bool> visy;
		double energy;
		vector<int> min_match;
		int min_n;

	};
	
	class Km
	{
	public:
		// KM algorithm

		/*构造函数：输入 Graph [方阵]，KM_eps阈值 以及误匹配阈值（可省）;*/
		Km(Graph graph, double eps0,double penalty0){
			gra = graph;
			eps = eps0;
			penalty = penalty0;
		}

		/*KM 主函数（二分图最小权匹配);*/
		void kmsolve();
		/*KM匹配能量，所选权之和;*/
		double energy();
		/*找增广路;*/
		bool findpath(int x);
		/*对应点集输出;*/
		int output(vector<int> &SP, vector<int> &TP, vector<int> &SPout, vector<int> &TPout);
		
		double penalty;
		double precision;
		double recall;

	protected:
		
	private:
		Graph gra;
		double eps;

	};
}
#endif