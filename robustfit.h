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

#ifndef ROBUSTFIT_H_
#define ROBUSTFIT_H_

int robustfit(double x[], double y[], int ndata, double *a, double *b, double *w);

void LSfit(double x[], double y[], int ndata, double *a, double *b,
		double *siga, double *sigb, double *chi2);
void WLSfit(double x[], double y[], double w[], int ndata, double *a, double *b,
		double *siga, double *sigb, double *chi2);

#endif /*ROBUSTFIT_H_*/
