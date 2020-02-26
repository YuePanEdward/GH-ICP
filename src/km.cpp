#include <cstring>
#include <cstdio>
#include <fstream>

#include "km.h"

using namespace std;

namespace ghicp
{

/*Find argumenting path*/
bool Km::findpath(int x)
{
	double tempDelta;
	gra.visx[x] = true;
	for (int y = 0; y < gra.n; ++y)
	{
		if (gra.visy[y])
			continue;
		tempDelta = gra.lx[x] + gra.ly[y] - gra.GTable[x][y];
		if (tempDelta < eps)
		{
			gra.visy[y] = true;
			if (gra.match[y] == -1 || findpath(gra.match[y]))
			{
				gra.match[y] = x;
				return true;
			}
		}
		else
		{
			gra.slack[y] = min(tempDelta, gra.slack[y]);
		}
	}
	return false;
}

/*KM main entrances*/
void Km::kmsolve()
{
	const int INF2 = 1000;
	//gra.energy = INF2;
	int size = gra.n;
	/*for (int n = 0; n < size- max(gra.sp, gra.tp); n++)
	{*/
	for (int i = 0; i < gra.n; i++)
	{
		gra.match[i] = -1;
		gra.ly[i] = 0;
	}

	for (int i = 0; i < gra.n; ++i)
	{
		gra.lx[i] = gra.GTable[i][0];
		for (int j = 0; j < gra.n; ++j)
		{
			gra.lx[i] = max(gra.GTable[i][j], gra.lx[i]);
		}
	}
	for (int x = 0; x < gra.n; ++x)
	{
		for (int j = 0; j < gra.n; ++j)
		{
			gra.slack[j] = INF2;
		}
		//if (x % 10 == 0) cout << "update" << x << endl;
		//int num = 0;
		while (true)
		{

			for (int i = 0; i < gra.n; i++)
			{
				gra.visx[i] = false;
				gra.visy[i] = false;
			}
			//num++;
			//cout << num<<" ";
			if (findpath(x))
				break;
			double delta = INF2;
			for (int j = 0; j < gra.n; ++j)
			{
				if (!gra.visy[j])
					delta = min(delta, gra.slack[j]);
			}
			for (int i = 0; i < gra.n; ++i)
			{
				if (gra.visx[i])
					gra.lx[i] -= delta;
			}
			for (int i = 0; i < gra.n; ++i)
			{
				if (gra.visy[i])
					gra.ly[i] += delta;
				else
					gra.slack[i] -= delta;
			}
			//slack[j] = min(lx[i] + ly[j] -w[i][j])
		}
	}
	/*double ans = 0;
		for (int i = 0; i < gra.n; ++i){
			if (gra.match[i] != -1) ans += gra.GTable[gra.match[i]][i];
		}
		ans = -ans;*/
	//cout << n << " energy: " << ans << endl;
	//gra.energy = min(ans,gra.energy);
	/*if (gra.energy > ans){
			gra.energy = ans;
			gra.min_n = gra.n;
			gra.min_match = gra.match;
		}*/
	//gra.energy = ans;
	/*gra.n--;
		gra.GTable.resize(gra.n, vector<double>(gra.n));
		gra.lx.resize(gra.n);
		gra.ly.resize(gra.n);
		gra.match.resize(gra.n);
		gra.slack.resize(gra.n);
		gra.visx.resize(gra.n);
		gra.visy.resize(gra.n);*/
	//}
	//cout << "min energy is: " << gra.energy << endl;
	//return gra.energy;
}

double Km::Calenergy()
{
	const int INF = 10000;
	int size = gra.n;
	gra.energy = 0;
	for (int i = 0; i < size; i++)
	{
		if (gra.GTable[gra.match[i]][i] != -INF)
		{
			gra.energy -= gra.GTable[gra.match[i]][i];
		}
	}
	return gra.energy;
}

/*Output the correspondence calculated from KM*/
int Km::output(vector<int> &SP, vector<int> &TP, vector<int> &SPout, vector<int> &TPout)
{
	//const int INF = 1000;
	ofstream ofs;
	ofs.open("Corres.txt");
	int cor_number = 0;
	int cor_exact_num = 0;
	if (ofs.is_open())
	{
		//F2 modified method
		ofs << gra.sp << "SP , " << gra.tp << "TP" << endl
			<< " SP  -  TP" << endl;
		int size = gra.n;
		for (int i = 0; i < size; i++)
		{
			if (gra.match[i] == i)
				cor_exact_num++;

			if (gra.GTable[gra.match[i]][i] != -penalty)
			{

				ofs << gra.match[i] << " - " << i << endl;
				SP.push_back(gra.match[i]);
				TP.push_back(i);
				cor_number++;
			}
			else
			{
				if (gra.sp >= gra.tp)
				{
					ofs << gra.match[i] << " - "
						<< "?" << endl;
					SPout.push_back(gra.match[i]);
					if (i < gra.tp)
					{
						ofs << "?"
							<< " - " << i << endl;
						TPout.push_back(i);
					}
				}
				else
				{
					ofs << "?"
						<< " - " << i << endl;
					TPout.push_back(i);
					if (gra.match[i] < gra.sp)
					{
						ofs << gra.match[i] << " - "
							<< "?" << endl;
						SPout.push_back(gra.match[i]);
					}
				}
			}
		}
		ofs.close();
	}
	/*
		//F1 initial method
		ofs << "min energy is " << gra.energy << endl<< gra.sp << "SP , " << gra.tp << "TP" << endl<<" SP  -  TP"<<endl;
		//for (size_t i = 0; i <gra.min_n; ++i){
		for (size_t i = 0; i <gra.n; ++i){
			if (gra.match[i] < gra.sp && i < gra.tp) { 
				
				ofs << gra.match[i] << " - " << i << endl; 
				SP.push_back(gra.match[i]);
				TP.push_back(i);
				cor_number++;
			}
			else if (gra.match[i] < gra.sp && i >= gra.tp)
			{
				ofs << gra.match[i] << " - " << "?" << endl;
				SPout.push_back(gra.match[i]);
			}
			else if (gra.match[i] >= gra.sp && i < gra.tp)
			{
				ofs << "?" << " - " << i << endl;
				TPout.push_back(i);
			}
		}
		ofs.close();
	}*/
	//cout << "Correspondence output" << endl;
	precision = 1.0 * cor_exact_num / cor_number;
	recall = 1.0 * cor_exact_num / gra.n;

	SP.resize(cor_number);
	TP.resize(cor_number);

	return cor_number;
}

} // namespace ghicp

//test for KM
/*vector<vector<double>> Weight(3, vector<double>(3));
Weight[0][0] = -5;
Weight[0][1] = -2;
Weight[0][2] = -100;
Weight[1][0] = -4;
Weight[1][1] = -2;
Weight[1][2] = -6;
Weight[2][0] = -100;
Weight[2][1] = -1;
Weight[2][2] = -7;
Graph graphtest;
graphtest.GTable = Weight;
graphtest.n = 3;
graphtest.lx.resize(3);
graphtest.ly.resize(3);
graphtest.match.resize(3);
graphtest.slack.resize(3);
graphtest.visx.resize(3);
graphtest.visy.resize(3);
cout << "Graoh OK" << endl;
Km kmsolver(graphtest);
double minenergy = kmsolver.kmsolve();
kmsolver.output(3, 3);*/