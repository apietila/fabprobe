/*
 This file is part of fabprobe.

 fabprobe is free software; you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation; either version 2 of the License, or
 (at your option) any later version.

 fabprobe is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with fabprobe; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
*/

/*-------------------------------------------------
   fabprobe : non-cooperative available bandwidth
              estimation tool
   Author   : Daniele Croce (daniele croce eurecom fr)
   Release  : Ver 0.9
--------------------------------------------------*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "nrutil.h"
#include "qr.h"
#include "robustfit.h"

#define D 1.4901e-08
#define TUNE 4.6850

//int main(int argc, char* argv[]){
//
//	double x[101]={-1, 1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39,40,41,42,43,44,45,46,47,48,49,50,51,52,53,54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69,70,71,72,73,74,75,76,77,78,79,80,81,82,83,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100};
//	double y[101]={-1, 86971,86007,85585,85862,86251,85348,86444,86812,85895,85731,86931,107204,105840,104457,102491,101925,99457,97793,96339,94962,92998,114488,134451,133081,131111,130471,129383,127222,125461,124096,122436,120770,119400,118440,138034,136355,135296,133292,131827,130669,128508,126853,126974,124318,122949,121287,120426,118167,116504,114834,113574,111750,110091,108429,106978,105818,103951,102694,100633,99170,97507,95972,94182,92822,91169,89504,88035,88558,86782,87920,87150,88279,87415,87812,86854,88079,88031,88349,87485,87122,87058,87993,87322,87113,87547,88777,87117,87549,87381,86520,87443,88674,88007,87845,87475,87107,87738,86845,88101,87212};
//	double y[100]={-1, 1856,604,1371,1724,0,1873,1456,863,537,2418,1215,1012,606,1502,1195,1497,2933,2132,1287,590,1190,1290,1671,2908,1083,1381,180,2064,864,697,998,1105,710,2914,1708,1708,1626,2034,1733,1361,1514,1809,2529,2010,1313,1710,713,1128,713,1011,2417,2405,1302,2622,1207,1603,2204,705,1560,1713,1273,4968,2673,1479,297,1483,285,581,2297,7584,5087,2574,1785,2182,274,561,963,1263,2479,1471,1070,1967,1465,2530,150,2838,845,1141,1546,1805,1409,1012,1418,2625,793,1223,705,1005,1405};
//
//	int ndata=100;
//	double *w;
//	double a, b;
//
//	w = dvector(1,ndata);
//
//	robustfit(x, y, ndata, &a, &b, w);
//
//	printf("a=%.4f b=%.4f\nPrinting w:\n", a, b);
//	printVector(w, ndata);
//
//	free_dvector(w, 1,ndata);
//
//	return 0;
//}


int robustfit(double x[], double y[], int ndata, double *a, double *b, double *w)
{
	int 	i, p=2, iter=0, iterlim=50;
	double 	a0=0, b0=0, s, tiny_s, r, siga, sigb, chi2;
	double 	**X, **Q, **R, **E, **invR;
	double 	*adjfactor, *radj;

	X = dmatrix(1,ndata, 1,p);
	Q = dmatrix(1,ndata, 1,p);
	R = dmatrix(1,p, 1,p);
	invR = dmatrix(1,p, 1,p);
	E = dmatrix(1,ndata, 1,p);

	adjfactor = dvector(1,ndata);
	radj = dvector(1,ndata);

	/* X=[ones(n,1) X]; */
	for (i=1; i<=ndata; i++)
	{
		X[i][1] = 1;
		X[i][2] = x[i];
		E[i][1] = 0.0;
		E[i][2] = 0.0;
	}

	QRDecomposition(X, ndata, p, Q, R);

	LSfit(x, y, ndata, a, b, &siga, &sigb, &chi2);

	//fprintf(stderr, "%.3f ", *b);

	inverse2(R, invR);
	computeE(X, invR, E, ndata); //E=X(:,[2 1])*inv(R);
	computeADJ(E, adjfactor, ndata); // h = min(.9999, sum((E.*E)')'); adjfactor = 1 ./ sqrt(1-h);

	// If we get a perfect or near perfect fit, the whole idea of finding
	// outliers by comparing them to the residual standard deviation becomes
	// difficult.  We'll deal with that by never allowing our estimate of the
	// standard deviation of the error term to get below a value that is a small
	// fraction of the standard deviation of the raw response values.
	tiny_s = 10e-6 * std(y, ndata);
	if (tiny_s==0.0)
	    tiny_s = 1.0;

	/* Perform iteratively reweighted least squares to get coefficient estimates */
	while((iter==0) || fabs(*a-a0)>D*DMAX(fabs(*a),fabs(a0))
					|| fabs(*b-b0)>D*DMAX(fabs(*b),fabs(b0)) ) // |a-a0|>D*max(|a|,|a0|) ?? same for b
	{
		if(++iter > iterlim)
		{
			//fprintf(stderr,"Warning: Iteration limit reached.\n");
			break;
		}

		/* Compute residuals from previous fit, then compute scale estimate */
		// r=y - X*b; radj=r.*adjfactor;
		for (i=1; i<=ndata; i++)
			radj[i] = adjfactor[i] * (y[i]-((*a)+X[i][2]*(*b)));

		s = madsigma(radj, ndata);

		/* Compute new weights from these residuals, then re-fit */
		// w=(abs(radj/(s*tune))<1) .* (1 - (radj/(s*tune)).^2).^2;
		for (i=1; i<=ndata; i++)
		{
			r = radj[i]/(DMAX(s,tiny_s)*TUNE);
			if (fabs(r)<1)
				w[i] = (1 - r*r) * (1 - r*r);
			else
				w[i] = 0;
		}

		a0=*a;
		b0=*b;

		WLSfit(x, y, w, ndata, a, b, &siga, &sigb, &chi2);
//		printf("a=%.4f b=%.4f\n", a0, b0);

	}

	free_dmatrix(X, 1,ndata, 1,p);
	free_dmatrix(Q, 1,ndata, 1,p);
	free_dmatrix(R, 1,p, 1,p);
	free_dmatrix(invR, 1,p, 1,p);
	free_dmatrix(E, 1,ndata, 1,p);

	free_dvector(adjfactor, 1,ndata);
	free_dvector(radj, 1,ndata);

	if (iter>iterlim)
		return 1;
	else
		return 0;
}


void LSfit(double x[], double y[], int ndata, double *a, double *b,
		double *siga, double *sigb, double *chi2)
/*
Given a set of data points x[1..ndata],y[1..ndata] fit them to a straight line y = a + bx
by minimizing chi2. Returned are a,b and their respective probable uncertainties siga and sigb,
the chi-square chi2. The standard deviations are assumed to be unavailable: the normalization
of chi2 is to unit standard deviation on all points.
*/
{
	int i;
	double t,sxoss,sx=0.0,sy=0.0,st2=0.0,ss,sigdat;
	*b=0.0;
	for (i=1;i<=ndata;i++) { // Accumulate sums without weights.
		sx += x[i];
		sy += y[i];
	}
	ss=ndata;
	sxoss=sx/ss;

	for (i=1;i<=ndata;i++) {
		t=x[i]-sxoss;
		st2 += t*t;
		*b += t*y[i];
	}

	*b /= st2; // Solve for a, b, siga, and sigb.
	*a=(sy-sx*(*b))/ss;
	*siga=sqrt((1.0+sx*sx/(ss*st2))/ss);
	*sigb=sqrt(1.0/st2);

	*chi2=0.0; // Calculate chi2.
	for (i=1;i<=ndata;i++)
		*chi2 += SQR(y[i]-(*a)-(*b)*x[i]);
	sigdat=sqrt((*chi2)/(ndata-2)); // For unweighted data evaluate typical sig using chi2, and adjust the standard deviations.
	*siga *= sigdat;
	*sigb *= sigdat;
}


void WLSfit(double x[], double y[], double w[], int ndata, double *a, double *b,
		double *siga, double *sigb, double *chi2)
/*
 * Weighted least square fit
 */
{
	int j;
	double del,sxy=0.0,sy=0.0,sx=0.0,sxx=0.0,s=0.0;

	for (j=1;j<=ndata;j++)
	{
		sx += x[j]*w[j];
		sy += y[j]*w[j];
		sxy += x[j]*y[j]*w[j];
		sxx += x[j]*x[j]*w[j];
		s += w[j];
	}
	del=s*sxx-sx*sx;
	*a=(sxx*sy-sx*sxy)/del; //Least-squares solutions.
	*b=(s*sxy-sx*sy)/del;
}
