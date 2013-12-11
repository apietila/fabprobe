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

#ifndef QR_H_
#define QR_H_

int ModifiedGramSchmidt(double **x, int rows, int cols, double **q, double **r);
int QRDecomposition(double **A, int rows, int cols, double **Q, double **R);
void inverse2(double **A, double **inv);

double Norm_l2(double **v, int col, int rows);
double dot(double **u, int colu, double **v, int colv, int rows);
void computeE(double **X, double **invR, double **E, int ndata);
void computeADJ(double **E, double *adjfactor, int ndata);
double std(double *x, int ndata);
double madsigma(double *r, int ndata);

void printVector(double *v, int ndata);
void printMatrix(double **m, int rows, int cols);

#endif /*QR_H_*/
