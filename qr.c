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


int ModifiedGramSchmidt(double **x, int rows, int cols, double **q, double **r)
{
	int i, j, k;
	double **y = dmatrix(1,rows, 1,cols);

	/* y=x */
	for (i=1; i<=rows; i++)
		for (j=1; j<=cols; j++)
			y[i][j]=x[i][j];


	for (j=1; j<=cols; j++)
	{
		r[j][j] = Norm_l2(y, j, rows);

		if (r[j][j] == 0.0)
		{
			free_dmatrix(y, 1,rows, 1,cols);
			return -1;
		}
		else
			/* q(:,j)=x(:,j)./r(j,j) */
			for (i=1; i<=rows; i++)
				q[i][j] = y[i][j]/r[j][j];

		for (k=j+1; k<=cols; k++)
		{
			r[j][k] = dot(y, k, q, j, rows);
			/* y = y - r(j,k).*q(:,j) */
			for (i=1; i<=rows; i++)
				y[i][k] -= r[j][k]*q[i][j];

		}
	}

	free_dmatrix(y, 1,rows, 1,cols);
	return 0;
}


int QRDecomposition(double **A, int rows, int cols, double **Q, double **R)
/* [Q,R,perm]=qr(X,0) */
{
	int i, j;
	int status;

	double **q = dmatrix(1,rows, 1,cols),
			**v = dmatrix(1,rows, 1,cols);

	for (i=1; i<=rows; i++)
		for (j=1; j<=cols; j++)
			v[i][j] = A[i][cols-(j-1)];

	status = ModifiedGramSchmidt(v, rows, cols, q, R);

	for (i=1; i<=rows; i++)
		for (j=1; j<=cols; j++)
			Q[i][j] = -q[i][j];

	for (i=1; i<=cols; i++)
		for (j=1; j<=cols; j++)
			R[i][j] = -R[i][j];

	free_dmatrix(q, 1,rows, 1,cols);
	free_dmatrix(v, 1,rows, 1,cols);

	return status;
}


double Norm_l2(double **v, int col, int rows)
{
	int i;
	double sum = 0.0;

	for (i=1; i<=rows; i++)
		sum += v[i][col]*v[i][col];

	return (sqrt(sum));
}


double dot(double **u, int colu, double **v, int colv, int rows)
/* u(:,colu)'*v(:,colv), i.e. sum(u(:,colu).*v(:,colv)) */
{
	int i;
	double sum = 0.0;

	for(i=1;i<=rows;i++)
		sum += u[i][colu]*v[i][colv];

	return sum;
}

void inverse2(double **A, double **inv)
/* compute inverse of a 2-by-2 matrix */
{
	double determinant = A[1][1]*A[2][2] - A[1][2]*A[2][1];
	inv[1][1] = A[2][2]/determinant;
	inv[1][2] = -A[1][2]/determinant;
	inv[2][1] = -A[2][1]/determinant;
	inv[2][2] = A[1][1]/determinant;
}

void computeE(double **X, double **invR, double **E, int ndata)
/* compute E=X(:,[2 1])*inv(R); */
{
	int i, j, k;
	for (i=1; i<=ndata; i++)
		for (j=1; j<=2; j++)
			for (k=1; k<=2; k++)
				E[i][j] += X[i][2-(k-1)]*invR[k][j];
}

void computeADJ(double **E, double *adjfactor, int ndata)
/* h = min(.9999, sum((E.*E)')'); adjfactor = 1 ./ sqrt(1-h); */
{
	int i;
	for (i=1; i<=ndata; i++)
		adjfactor[i] = (E[i][1]*E[i][1] + E[i][2]*E[i][2]) > 0.9999? 1.0/sqrt(1-0.9999)
				:1.0/sqrt(1 - (E[i][1]*E[i][1] + E[i][2]*E[i][2]));
}

double std(double *x, int ndata)
/* (Unbiased) standard deviation of x */
{
	int i;
	double mean=0.0, variance=0.0;

	for (i=1; i<=ndata; i++)
		mean += x[i];
	mean /= ndata;

	for (i=1; i<=ndata; i++)
		variance += (x[i]-mean)*(x[i]-mean);
	variance /= (ndata-1);

	return sqrt(variance);
}

/* Sort compare function for qsort function */
int compare_double(const void *v1, const void *v2)
{
	double *x1,*x2;
	x1 = (double *)v1;
	x2 = (double *)v2;
        if(*x1<*x2)return (-1);
        else if(*x1==*x2) return (0);
        else return(1);
}

double madsigma(double *r, int ndata)
/* Compute sigma estimate using MAD (Median Absolute Deviation) of residuals from 0 */
{
	int i;
	double *rs, s=0.0;

	//rs = sort(abs(r));
	rs = dvector(1,ndata);
	rs[0]=-1;
	for (i=1; i<=ndata; i++)
		rs[i] = fabs(r[i]);

	/* sort rs using the qsort function */
	qsort((void *)rs, ndata+1, sizeof(double), compare_double);

	//s = median(rs(2:end)) / 0.6745;
	/* These are done exactly using formulas appropriate for small samples*/
	if((ndata-1)%2)  /* this is case for odd number */
		s = rs[2+(ndata-1)/2];
	else
		s = (rs[1+(ndata-1)/2]+rs[2+(ndata-1)/2])/2.0;

	free_dvector(rs, 1,ndata);
	return s/0.6745;
}

void printVector(double *v, int ndata)
{
	int i;
	for(i=1;i<=ndata;i++)
		printf("%d:\t%.4f\n", i, v[i]);
}

void printMatrix(double **m, int rows, int cols)
{
	int i,j;
	for(i=1;i<=rows;i++)
	{
		printf("%d:", i);
		for (j=1; j<=cols; j++)
			printf("\t%.4f", m[i][j]);
		printf("\n");
	}
}
