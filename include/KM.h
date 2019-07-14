#ifndef KM
#define KM

#include <Eigen/Dense>
#include <vector>
#include "utility.h"

using namespace utility;
using namespace std;

namespace KMSpace{
	
	struct Graph{  //����ͼ�ṹ��;
		vector<vector<double>> GTable; //�ڽӾ��� 
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

		/*���캯�������� Graph [����]��KM_eps��ֵ �Լ���ƥ����ֵ����ʡ��;*/
		Km(Graph graph, double eps0,double penalty0){
			gra = graph;
			eps = eps0;
			penalty = penalty0;
		}

		/*KM ������������ͼ��СȨƥ��);*/
		void kmsolve();
		/*KMƥ����������ѡȨ֮��;*/
		double energy();
		/*������·;*/
		bool findpath(int x);
		/*��Ӧ�㼯���;*/
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