#include <math.h>


#include<math.h>
   
#define GENER 3

#define IM1 2147483563L
#define IM2 2147483399L
#define AM (1.0/IM1)
#define IMM1 (IM1-1)
#define IA1 40014L
#define IA2 40692L
#define IQ1 53668L
#define IQ2 52774L
#define IR1 12211L
#define IR2 3791L
#define NTAB 32
#define NDIV (1+IMM1/NTAB)
#define EPS 1.0e-10
#define RNMX (1.0-EPS)

static long int s1[GENER]={1,1,1};
static long int s2[GENER]={123456789L,123456789L,123456789L};
static long int iy[GENER]={0,0,0};
static long int iv[GENER][NTAB];




void SetSeed(long int s,int type);

double Rand01(int type);

double Gauss(double m,double sigma,int type);

double Pareto(double alpha, double beta, int type);