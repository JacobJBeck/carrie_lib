#pragma once
/**
This class analyzes the reward signal given to the agent, over a run. Static class.
*/

#include <vector>
using namespace std;

typedef vector<double> matrix1d;


class RewardAnalysis
{
public:
	RewardAnalysis(void);
	static double u(double d){
		if (d>0.0) return 1.0;
		else return 0.0;
	}
	static double Fgi(matrix1d gi, matrix1d gi_prime, matrix1d G, matrix1d G_prime){
		size_t Z = gi.size(); // this should be same size for all
		double factoredness = 0.0;
		for (int z=0; z<Z; z++){
			for (int z_prime; z_prime<Z; z_prime++){
				factoredness += u((gi[z]-gi_prime[z_prime])*(G[z]-G_prime[z_prime]));
			}
		}
		return factoredness/double(Z);
	}
	~RewardAnalysis(void);
};

