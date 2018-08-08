#ifndef KM
#define KM

#include<pcl\point_cloud.h>
#include<pcl\point_types.h>
#include <Eigen/dense>
#include <vector>
#include "utility.h"

using namespace utility;
using namespace std;

namespace KMSpace{
	struct Graph{
		vector<vector<double>> GTable; //ÁÚ½Ó¾ØÕó 
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

		Km(Graph graph, double eps0,double penalty0){
			gra = graph;
			eps = eps0;
			penalty = penalty0;
		}
		int output(vector<int> &SP, vector<int> &TP, vector<int> &SPout, vector<int> &TPout);
		void kmsolve();
		double energy();
		double penalty;
		bool findpath(int x);

	protected:
		
	private:
		Graph gra;
		double eps;

	};
}
#endif