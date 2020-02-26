#ifndef _INCLUDE_KM_H_
#define _INCLUDE_KM_H_

//Eigen
#include <Eigen/Core>

#include <vector>
#include "utility.h"

using namespace std;

namespace ghicp
{

struct Graph
{								   //Bipartite graph;
	vector<vector<double>> GTable; //The adjacency matrix
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

	/*Constructor: Input Graph, KM_eps threshold and prior mismatch threshold*/
	Km(Graph graph, double eps0, double penalty0)
	{
		gra = graph;
		eps = eps0;
		penalty = penalty0;
	}

	/*KM main entrance;*/
	void kmsolve();
	/*Calculate the KM energy, the sum of all the selected edges' weight*/
	double Calenergy();
	/*Find the augmenting path*/
	bool findpath(int x);
	/*Output correspondences*/
	int output(vector<int> &SP, vector<int> &TP, vector<int> &SPout, vector<int> &TPout);

	double penalty;
	double precision;
	double recall;

  protected:
  private:
	Graph gra;
	double eps;
};
} // namespace ghicp
#endif //_INCLUDE_KM_H_