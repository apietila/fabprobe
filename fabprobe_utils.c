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

#include "fabprobe_gbls.h"
#include "fabprobe_snd.h"
#include "nrutil.h"
#include "qr.h"
#include "robustfit.h"

#define INCR    1
#define NOTR    2
#define DISCARD 3
#define UNCL    4


int get_trend(double owdfortd[], l_int32 pkt_cnt) {
	double median_owd[MAX_STREAM_LEN];
	l_int32 median_owd_len=0;
	double ordered[MAX_STREAM_LEN];
	l_int32 j, count, pkt_per_min;
	//pkt_per_min = 5 ;
	pkt_per_min = (int)floor(sqrt((double)pkt_cnt));
	count = 0;
	for (j = 0; j < pkt_cnt; j=j+pkt_per_min) {
		if (j+pkt_per_min >= pkt_cnt)
			count = pkt_cnt - j;
		else
			count = pkt_per_min;
		order_dbl(owdfortd, ordered, j, count);
		if (count % 2 == 0)
			median_owd[median_owd_len++] =( ordered[(int)(count*.5) -1] + ordered[(int)(count*0.5)] )/2;
		else
			median_owd[median_owd_len++] = ordered[(int)(count*0.5)];
	}
	pct_metric[trend_idx]=pairwise_comparision_test(median_owd, 0,
			median_owd_len);
	pdt_metric[trend_idx]=pairwise_diff_test(median_owd, 0, median_owd_len);

	/***************************************************************/
	/* If NO TREND, double check after eliminating decreasing RTTs */
	/* Current implementation takes into account only consecutive decreasing
	/* RTT without other checking */
	/***************************************************************/
#define MIN_DECREASING_SAMPLES 8

/* use this instead of MIN_DECREASING_SAMPLES if you want to relate
 * the number of bad samples with the gap size. More complicated, possibly
 * inaccurate. Interesting cross-check however! */
//#define MAX_GAP_SIZE 8  /* # of affected packets after which we filter the traffic */

	l_int32 pct = DISCARD, pdt = DISCARD;
	double owdfortd_filtered[MAX_STREAM_LEN];
	double rcvIAT[MAX_STREAM_LEN-1];
	l_int32 filtered_pkt_cnt = 0;
	int b2b_start=0, last_included = -1;
	int i, k, decreasing=1;
#ifdef MAX_GAP_SIZE
	int max_gap = MAX_GAP_SIZE*time_interval;
#endif

	if (pct_metric[trend_idx] > 1.1 * PCT_THRESHOLD) {
		pct = INCR;
	} else if (pct_metric[trend_idx] <= PCT_THRESHOLD*1.1 && pct_metric[trend_idx] >= PCT_THRESHOLD*.9) {
		pct = UNCL;
	}
	if (pdt_metric[trend_idx] > 1.1 * PDT_THRESHOLD) {
		pdt = INCR;
	} else if (pdt_metric[trend_idx] <= PDT_THRESHOLD*1.1 && pdt_metric[trend_idx] >= PDT_THRESHOLD*.9) {
		pdt = UNCL;
	}

	if(!((pct == INCR && pdt == INCR) ||
		 (pct == INCR && pdt == UNCL) ||
		 (pct == UNCL && pdt == INCR) ))
	{

		if ( !quiet)
			printf("\nNo trend (pct=%3.2f, pdt=%3.2f): searching descending RTTs\n", pct_metric[trend_idx], pdt_metric[trend_idx]);

		/* Compute inter-arrival time (IAT) of received RSTs */
		/* for simplicity, rcvIAT~(owd2-owd1)+time_interval
		 * (true only if sndIAT==time_interval EXACTLY) */
		for (i = 0; i < pkt_cnt-1; i++) {
			rcvIAT[i] = owdfortd[i+1]-owdfortd[i] + time_interval;
		}

#ifdef MAX_GAP_SIZE
		/* Search for big gaps (forget about the last MAX_GAP_SIZE/2 packets) */
		for (i = 0; i < pkt_cnt-1-MAX_GAP_SIZE/2; i++)
		{
			int affected_packets = 0;
			if(rcvIAT[i] < max_gap)
			{
				owdfortd_filtered[filtered_pkt_cnt++] = owdfortd[i];
				last_included = i;
			}
			else
			{
				affected_packets += rcvIAT[i]/time_interval;
				/* minimum number, more could be possible! */

				/* ERROR!!! time_interval is NOT the IAT at which we should receive the packets
				 * but the IAT at which we have sent them! */
				k = i;
				/* Verify that a descending trend follows */
				while (affected_packets!=0 && k!=pkt_cnt-1)
				{
					k++;
					if(b2b_start<=i)
						b2b_start = k;

					if(rcvIAT[k] > max_gap)
						affected_packets += rcvIAT[k]/time_interval;
					else if(owdfortd[k+1]-owdfortd[k]<0.0)
					{
						if ( k-b2b_start == affected_packets ) /* found all affected packets */
						{
#ifdef DEBUG
							printf("(Complete)Eliminating %d packets (%d to %d):\n", k-b2b_start, b2b_start, k);
							for(j = b2b_start; j <= k; j++)
								printf("owd[%d]=%.0f\n", j, owdfortd[j]);
#endif
							for(j = last_included+1; j < b2b_start; j++)
								owdfortd_filtered[filtered_pkt_cnt++] = owdfortd[j];
							last_included = k; /* jump from b2b_start to k */
							affected_packets = 0;
							b2b_start = 0;
						}
					}
					else
					{
						if (decreasing) /* ignore not decreasing if only one sample is "wrong" */
						{
							decreasing--;
							if ( k-b2b_start == affected_packets ) /* found all affected packets */
							{
#ifdef DEBUG
								printf("(Complete)Eliminating %d packets (%d to %d):\n", k-b2b_start, b2b_start, k);
								for(j = b2b_start; j <= k; j++)
									printf("owd[%d]=%.0f\n", j, owdfortd[j]);
#endif
								for(j = last_included+1; j < b2b_start; j++)
									owdfortd_filtered[filtered_pkt_cnt++] = owdfortd[j];
								last_included = k; /* jump from b2b_start to k */
								affected_packets = 0;
								b2b_start = 0;
							}
						}
						else
						{
							decreasing = 1;
							if ( k-b2b_start > MAX_GAP_SIZE*2/3 ) /* do not eliminate less than MAX_GAP_SIZE*2/3 packets */
							{
#ifdef DEBUG
								printf("(Partial) Eliminating %d packets (%d to %d):\n", k-b2b_start, b2b_start, k);
								for(j = b2b_start; j <= k; j++)
									printf("owd[%d]=%.0f\n", j, owdfortd[j]);
#endif
								for(j = last_included+1; j < b2b_start; j++)
									owdfortd_filtered[filtered_pkt_cnt++] = owdfortd[j];
								last_included = k; /* jump from b2b_start to k */
								affected_packets-=k-b2b_start;
								b2b_start = 0;
								if (affected_packets<MAX_GAP_SIZE/2)
								{
									affected_packets = 0;
									break;
								}
							}
							else
							{
								for(j = last_included+1; j <= k; j++)
									owdfortd_filtered[filtered_pkt_cnt++] = owdfortd[j];
								last_included = k;
								b2b_start = 0;
							}
						}
					}
				}
				i = k;
			}
		}
#else
		/* Search for consecutive samples with decreasing RTT */
		for (i = 0; i < pkt_cnt-1; i++)
		{
			k = i;
			decreasing = 1;
			/* Verify that a descending trend follows */
			while (decreasing>=0 && k!=pkt_cnt-1)
			{
				if(b2b_start<i)
					b2b_start = k;

				if(owdfortd[k+1]-owdfortd[k]>=0.0)
				{
					if (decreasing) /* ignore not decreasing if only one sample is "wrong" */
						if(k>i)
							decreasing--;
						else
							decreasing = -1; //first packet
					else
					{
						decreasing = -1;
						if ( k-b2b_start > MIN_DECREASING_SAMPLES ) /* do not eliminate less than MIN_DECREASING_SAMPLES packets */
						{
#ifdef DEBUG
							printf("Eliminating %d packets (%d to %d):\n", k-b2b_start, b2b_start, k);
							for(j = b2b_start; j <= k; j++)
								printf("owd[%d]=%.0f\n", j, owdfortd[j]);
#endif

							/* eliminate all samples except last 3 (give a margin) */
							for(j = last_included+1; j < b2b_start; j++)
								owdfortd_filtered[filtered_pkt_cnt++] = owdfortd[j];
							last_included = k-3; /* jump from b2b_start to k-3 */
							b2b_start = 0;
						}
						else
						{
							/* not enough b2b samples. Add all of them */
							for(j = last_included+1; j <= k; j++)
								owdfortd_filtered[filtered_pkt_cnt++] = owdfortd[j];
							last_included = k;
							b2b_start = 0;
						}
					}
				}
				k++;
			}
			if (last_included>i)
				i = last_included;
		}
#endif
		/* add last packets */
		for (i=last_included+1; i < pkt_cnt; i++)
			owdfortd_filtered[filtered_pkt_cnt++] = owdfortd[i];

		if ( !quiet)
			printf("==> %d samples eliminated (%d left)\nRecomputing trend: ", pkt_cnt-filtered_pkt_cnt, filtered_pkt_cnt);

		if(filtered_pkt_cnt<MIN_STREAM_LEN)
			return filtered_pkt_cnt;

		/*************************/
		/* trend after filtering */
		/*************************/
		median_owd_len=0;
		pkt_per_min = (int)floor(sqrt((double)filtered_pkt_cnt));
		count = 0;
		for (j = 0; j < filtered_pkt_cnt; j=j+pkt_per_min) {
			if (j+pkt_per_min >= filtered_pkt_cnt)
				count = filtered_pkt_cnt - j;
			else
				count = pkt_per_min;
			order_dbl(owdfortd_filtered, ordered, j, count);
			if (count % 2 == 0)
				median_owd[median_owd_len++] =( ordered[(int)(count*.5) -1] + ordered[(int)(count*0.5)] )/2;
			else
				median_owd[median_owd_len++] = ordered[(int)(count*0.5)];
		}
		pct_metric[trend_idx]=pairwise_comparision_test(median_owd, 0,
				median_owd_len);
		pdt_metric[trend_idx]=pairwise_diff_test(median_owd, 0, median_owd_len);

		/* done */
		if (pct_metric[trend_idx] > 1.1 * PCT_THRESHOLD) {
			pct = INCR;
		} else if (pct_metric[trend_idx] >= PCT_THRESHOLD*.9) {
			pct = UNCL;
		}
		if (pdt_metric[trend_idx] > 1.1 * PDT_THRESHOLD) {
			pdt = INCR;
		} else if (pdt_metric[trend_idx] >= PDT_THRESHOLD*.9) {
			pdt = UNCL;
		}

		if(!((pct == INCR && pdt == INCR) ||
			(pct == INCR && pdt == UNCL) ||
			(pct == UNCL && pdt == INCR) ))

			if ( !quiet)
				printf("trend confirmed ");
		else
			if ( !quiet)
				printf("trend changed!! ");

		if ( !quiet)
			printf("(pct=%3.2f, pdt=%3.2f).\n", pct_metric[trend_idx], pdt_metric[trend_idx]);


		/***********************************************************/
		/* end Decreasing Trend filtering */
		/***********************************************************/

		/***********************************************************/
		/* If still NO TREND, double check using Robust Regression */
		/***********************************************************/
		if(!((pct == INCR && pdt == INCR) ||
			(pct == INCR && pdt == UNCL) ||
			(pct == UNCL && pdt == INCR) ))
		{

			int limit_reached = 0;
			double owdfortd_filtered2[MAX_STREAM_LEN];
			l_int32 filtered_pkt_cnt2 = 0;

			if ( !quiet)
				printf("Cross-checking with robust regression.\n");

	/*	double inTput, outTput, rcv_time_interval;
		inTput=cur_pkt_sz * 8.0 /time_interval;
		rcv_time_interval=filtered_pkt_cnt*time_interval+owdfortd_filtered[filtered_pkt_cnt-1]-owdfortd_filtered[0];
		outTput=filtered_pkt_cnt*cur_pkt_sz * 8.0 /rcv_time_interval;

		/*r=robust		PCT	  PDT	inTput	outTput	*/
	/*	fprintf(stderr, "%.3f %.3f %.3f %.3f ", pct_metric[trend_idx], pdt_metric[trend_idx], inTput, outTput);
	*/
			double a, b;
			double *x, *y, *w;

			x = dvector(1,(int) filtered_pkt_cnt);
			y = dvector(1,filtered_pkt_cnt);
			w = dvector(1,filtered_pkt_cnt);

			for (j = 1; j <= filtered_pkt_cnt; ++j)
			{
				x[j] = j;
				y[j] = owdfortd_filtered[j-1];
			}

			limit_reached = robustfit(x, y, filtered_pkt_cnt, &a, &b, w);

//			printf("Printing w:\n");
//			printVector(w, filtered_pkt_cnt);
//			printf("Printing RTT samples:\n");
//			for(j = 0; j < filtered_pkt_cnt; ++j)
//			{
//				printf("%.0f\n", owdfortd_filtered[j]);
//			}
//			printf("Eliminationg w<0.1:\n");

			for(j = 1; j <= filtered_pkt_cnt; ++j)
				if(w[j]>=0.1)
					owdfortd_filtered2[filtered_pkt_cnt2++] = owdfortd_filtered[j-1];
//				else
//					printf("Sample %d: w=%.4f\n", j, w[j]);

			if ( !quiet)
			{
				if (limit_reached)
					printf("Warning: iteration limit reached.\n");
				printf("==> %d samples eliminated (%d left)\nRecomputing trend: ", filtered_pkt_cnt-filtered_pkt_cnt2, filtered_pkt_cnt2);
			}
			if(filtered_pkt_cnt2<MIN_STREAM_LEN)
				return filtered_pkt_cnt2;

			/*r=robust		trend:PCT-PDT-all	*/
			//fprintf(stderr, "%d %d %d ", pct, pdt, overall);

			/*************************/
			/* trend after filtering */
			/*************************/
			median_owd_len=0;
			pkt_per_min = (int)floor(sqrt((double)filtered_pkt_cnt2));
			count = 0;
			for (j = 0; j < filtered_pkt_cnt2; j=j+pkt_per_min) {
				if (j+pkt_per_min >= filtered_pkt_cnt2)
					count = filtered_pkt_cnt2 - j;
				else
					count = pkt_per_min;
				order_dbl(owdfortd_filtered2, ordered, j, count);
				if (count % 2 == 0)
					median_owd[median_owd_len++] =( ordered[(int)(count*.5) -1] + ordered[(int)(count*0.5)] )/2;
				else
					median_owd[median_owd_len++] = ordered[(int)(count*0.5)];
			}
			pct_metric[trend_idx]=pairwise_comparision_test(median_owd, 0,
					median_owd_len);
			pdt_metric[trend_idx]=pairwise_diff_test(median_owd, 0, median_owd_len);

			/* done */
			if (pct_metric[trend_idx] > 1.1 * PCT_THRESHOLD) {
				pct = INCR;
			} else if (pct_metric[trend_idx] >= PCT_THRESHOLD*.9) {
				pct = UNCL;
			}
			if (pdt_metric[trend_idx] > 1.1 * PDT_THRESHOLD) {
				pdt = INCR;
			} else if (pdt_metric[trend_idx] >= PDT_THRESHOLD*.9) {
				pdt = UNCL;
			}

			if(!((pct == INCR && pdt == INCR) ||
				(pct == INCR && pdt == UNCL) ||
				(pct == UNCL && pdt == INCR) ))

				if ( !quiet)
					printf("trend confirmed ");
			else
				if ( !quiet)
					printf("trend changed!!! ");

			if ( !quiet)
				printf("(pct=%3.2f, pdt=%3.2f).\n\n", pct_metric[trend_idx], pdt_metric[trend_idx]);

			free_dvector(x, 1,MAX_STREAM_LEN);
			free_dvector(y, 1,MAX_STREAM_LEN);
			free_dvector(w, 1,MAX_STREAM_LEN);
		}
			/*r=robust		rPCT	rPDT	rSlope	trend:rPCT-rPDT-all	*/
			//fprintf(stderr, "%.3f %.3f %.3f %d %d %d\n", pct_metric[trend_idx], pdt_metric[trend_idx], b, pct, pdt, overall);

		/***********************************************************/
		/* end Robust Regression filtering */
		/***********************************************************/
	}

	trend_idx+=1;

	return pkt_cnt;
}

/*
 depending upon trend in fleet :-
 - update the state variables.
 - decide the next fleet rate
 return -1 when converged
 */
l_int32 rate_adjustment(l_int32 flag) {
	l_int32 ret_val = 0;
	if (flag == INCREASING) {
		if (max_rate_flag)
			max_rate_flag=0;
		if (grey_max >= tr)
			grey_max = grey_min = 0;
		tr_max = tr;
		if(tr < tr_min)
			tr_min = 0;
		if (!converged_gmx_rmx_tm) {
			if (!converged())
				radj_increasing();
			else
				ret_val=-1; //return -1;
		} else {
			exp_flag = 0;
			if (!converged())
				radj_notrend();
		}
	} else if (flag == NOTREND) {
		if (grey_min < tr)
			grey_min = 0;
		if (grey_max < tr)
			grey_max = grey_min = 0;
		if (tr > tr_min)
		{
			tr_min = tr;
			if(tr > tr_max)
				tr_max = 0;
		}
		if (!converged_gmn_rmn_tm && !converged())
			radj_notrend();
		else
			ret_val=-1; //return -1 ;
	} else if (flag == GREY) {
		if (grey_max == 0 && grey_min == 0)
			grey_max = grey_min = tr;
		if (tr==grey_max || tr>grey_max) {
			grey_max = tr;
			if(tr > tr_max)
				tr_max = 0;
			if (!converged_gmx_rmx_tm) {
				if (!converged())
					radj_greymax();
				else
					ret_val=-1; //return -1 ;
			} else {
				exp_flag = 0;
				if (!converged())
					radj_notrend();
				else
					ret_val=-1;
			}
		} else if (tr < grey_min || grey_min == 0) {
			grey_min = tr;
			if(tr < tr_min)
				tr_min = 0;
			if (!converged())
				radj_greymin();
			else
				ret_val=-1; //return -1 ;
		}
	}

	if ( !quiet) {
		printf("  Rmin-Rmax             :: %.2f-%.2fMbps\n", tr_min, tr_max);
		printf("  Gmin-Gmax             :: %.2f-%.2fMbps\n", grey_min, grey_max);
	}

	if (ret_val == -1)
		return -1;
	if (tr >= max_rate)
		max_rate_flag++;

	if (max_rate_flag > 1)
		return -1;
	if (min_rate_flag > 1)
		return -1;
	transmission_rate = (l_int32) rint(1000000 * tr );
	return 0;
}

/*
 Calculate next fleet rate
 when fleet showed NOTREND trend.
 */
void radj_notrend() {
	if(equal(tr, adr*.75))
		tr=adr*.9;
	else if(equal(tr, adr*.25))
		tr=adr*.5;
	else
		terminate_gracefully(exp_start_time);
}

/*
 Calculate next fleet rate
 when fleet showed INCREASING trend.
 */
void radj_increasing() {
	if(equal(tr,adr*.75))
		tr=adr*.25;
	else if(equal(tr,adr*.25))
		tr=adr*.10;
	else
		terminate_gracefully(exp_start_time);
}

/*
 calculates fleet param L,T .
 calc_param returns -1, if we have
 reached to upper/lower limits of the
 stream parameters like L,T .
 otherwise returns 0 .
 */
l_int32 calc_param() {
	double tmp_tr;
	l_int32 tmp;
	l_int32 tmp_time_interval;
	if (tr < 150) {
		time_interval = 80>min_time_interval ? 80 : min_time_interval;
		cur_pkt_sz=rint(tr*time_interval/8.);
		if (cur_pkt_sz < min_pkt_size) {
			cur_pkt_sz = min_pkt_size;
			time_interval =rint((cur_pkt_sz)*8./tr);
			tr = ( cur_pkt_sz )*8. /time_interval;
		} else if (cur_pkt_sz > max_pkt_sz) {
			cur_pkt_sz = max_pkt_sz;
			time_interval = rint((cur_pkt_sz)*8./tr) > min_time_interval ?
					rint((cur_pkt_sz)*8./tr) : min_time_interval;
			tmp_tr = ( cur_pkt_sz )*8. /time_interval;
			if (equal(tr, tmp_tr))
				tr = tmp_tr;
			else
				return -1;
		}
	} else if (tr < 600) {
		tmp_tr = tr;
		tmp_time_interval = rint(( max_pkt_sz )* 8 / tr);
		if (cur_pkt_sz == max_pkt_sz && tmp_time_interval == time_interval)
			return -1;
		time_interval = tmp_time_interval;
		tmp=rint(tr*time_interval/8.);
		cur_pkt_sz=tmp<max_pkt_sz ? tmp : max_pkt_sz;
		tr = ( cur_pkt_sz ) *8./time_interval;
		if ((tr_min && (equal(tr,tr_min) || tr<tr_min))|| (grey_max && tmp_tr>grey_max && (equal(tr,grey_max) || tr<grey_max))) {
			do {
				--time_interval;
				cur_pkt_sz=rint(tr*time_interval/8.);
			} while (cur_pkt_sz > max_pkt_sz);
			tr = ( cur_pkt_sz ) *8./time_interval;
		}
	} else {
		cur_pkt_sz = max_pkt_sz;
		time_interval = rint(( cur_pkt_sz )* 8 / tr);
		tr = ( cur_pkt_sz ) *8./time_interval;
		if ((tr_min && (equal(tr,tr_min) || tr<tr_min))) {
			return -1;
		}
		if (equal(tr, tr_max)) {
			tr_max = tr;
			if (grey_max) {
				converged_gmx_rmx_tm=1;
				if (!converged_gmn_rmn && !converged_gmn_rmn_tm)
					radj_notrend();
				else
					return -1;
			} else {
				converged_rmn_rmx=1;
				return -1;
			}
		}
	}
	return 0;
}

/*
 Calculate next fleet rate
 when fleet showed GREY trend.
 */
void radj_greymax() {
	if(equal(tr,adr*.75))
		tr=adr*.5;
	else if(equal(tr,adr*.5) && tr_min==0)
		tr=adr*.25;
	else if(equal(tr,adr*.25) && grey_max==0)
		tr=adr*.5;
	else
		terminate_gracefully(exp_start_time);
}

/*
 Calculate next fleet rate
 when fleet showed GREY trend.
 */
void radj_greymin() {
	if(equal(tr,adr*.75))
		tr=adr*.5;
	else if(equal(tr,adr*.5) && tr_min==0)
		tr=adr*.25;
	else if(equal(tr,adr*.25) && grey_max==0)
		tr=adr*.5;
	else
		terminate_gracefully(exp_start_time);
}


void terminate_gracefully(struct timeval exp_start_time) {
	l_int32 ctr_code;
	char ctr_buff[8], buff[26];
	struct timeval exp_end_time;
	double min=0, max=0, min_percent=-1, max_percent=-1;

	double percentile_up, percentile_down;
	double var_up = 0.0, var_down = 0.0;
	double varmed_up = 0.0, varmed_down = 0.0;
	int i;

	gettimeofday(&exp_end_time, NULL);
	strncpy(buff, ctime(&(exp_end_time.tv_sec)), 24);
	buff[24] = '\0';
	if ( !quiet)
		printf("\n\t*****  RESULT *****\n");

	if(check_availbw)
	{
	if (congested_uplink) {
		if ( !quiet) {
			printf("Uplink (or downlink) heavily congested, unable to measure avail-bw.\n\n");
		}
	} else if (min_rate_flag) {
		if ( !quiet) {
			printf("Avail-bw < minimum sending rate.\n");
			printf("Increase MAX_TIME_INTERVAL in pathload_rcv.h from 200000 usec to a higher value.\n");
		}
	} else if (max_rate_flag && !interrupt_coalescence) {
		if ( !quiet) {
			printf("Avail-bw > maximum sending rate.\n");
			if (tr_min)
				printf("Avail-bw > %.2f (Mbps)\n", tr_min);
		}

	} else if (bad_fleet_cs && !interrupt_coalescence) {
		if ( !quiet)
			printf("Measurement terminated due to frequent CS @ sender/receiver.\n");
		if ((tr_min&& tr_max) || (grey_min&&grey_max)) {
			if (grey_min&& grey_max) {
				min = grey_min;
				max = grey_max;
			} else {
				min = tr_min;
				max = tr_max;
			}
			if ( !quiet) {
				printf("Available bandwidth range : %.2f - %.2f (Mbps)\n", min,
						max);
				printf("Measurements finished at %s \n", buff);
				printf("Measurement latency is %.2f sec \n", time_to_us_delta(
						exp_start_time, exp_end_time)/ 1000000);
			}
		}
	} else {
		if (!interrupt_coalescence && ((converged_gmx_rmx_tm && converged_gmn_rmn_tm) || converged_rmn_rmx_tm )) {
			if ( !quiet)
				printf("Actual probing rate != desired probing rate.\n");
			if (converged_rmn_rmx_tm) {
				min = tr_min;
				max = tr_max;
			} else {
				min = grey_min;
				max = grey_max;
			}
		} else if (!interrupt_coalescence && converged_rmn_rmx) {
			if ( !quiet)
				printf("User specified bandwidth resolution achieved\n");
			min = tr_min;
			max = tr_max;
		} else if (!interrupt_coalescence && converged_gmn_rmn && converged_gmx_rmx) {
			if ( !quiet)
				printf("Exiting due to grey bw resolution. Grey bw range: %.2f - %.2f\n",
						grey_min, grey_max);
			min = tr_min;
			max = tr_max;
		} else {
			min = tr_min;
			max = tr_max;
		}

		if ( !quiet) {
			if (lower_bound) {
				printf("Receiver NIC has interrupt coalescence enabled\n");
				printf("Available bandwidth is greater than %.2f (Mbps)\n", min);
			} else
				printf("Available bandwidth range : %.2f - %.2f (Mbps)\n", min,
						max);
			if(equal(min,adr*.9))
			{
				printf("avail-bw > 90%% ADR.\n\n");
				min_percent=90;
				max_percent=100;
			}
			else if(min==0)
			{
				if(max!=0 && !lower_bound)
				{
					printf("avail-bw < 25%% ADR.\n\n");
					min_percent=0;
					max_percent=25;
				}
				else
				{
					printf("Unable to measure the avail-bw.\n\n");
					min_percent=-1;
					max_percent=-1;
				}
			}
			else
			{
				printf("%.0f%% < avail-bw < %.0f%% ADR.\n\n", min*100.0/adr, (max==0?adr:max)*100.0/adr);
				min_percent= min*100.0/adr;
				max_percent= (max==0?adr:max)*100.0/adr;
			}
		}
	}
	}

	if(num_down!=-1)
	{
		/* compute median */
		/* percent-th quantile with linear interpolation */
		double percent = .5; //median!
		double ord_adr_up[NUM_ADR], ord_adr_down[NUM_ADR];
		double index, reminder;
		index = (num_up-1)*percent;
		reminder = (num_up-1)*percent - floor((num_up-1)*percent);
		order_dbl(adr_up_good, ord_adr_up, 0, num_up);
		percentile_up = ord_adr_up[(int)index]*(1.0-reminder) + ord_adr_up[(int)index+1]*reminder;

		index = (num_down-1)*percent;
		reminder = (num_down-1)*percent - floor((num_down-1)*percent);
		order_dbl(adr_down_good, ord_adr_down, 0, num_down);
		percentile_down = ord_adr_down[(int)index]*(1.0-reminder) + ord_adr_down[(int)index+1]*reminder;

		/* compute variance */
		if(num_up>1)
		{
			for (i = 0; i < num_up; ++i) {
				var_up += (adr_up_good[i]-adr_uplink)*(adr_up_good[i]-adr_uplink);
			}
			var_up /=num_up-1;
		}
		if(num_down>1)
		{
			for (i = 0; i < num_down; ++i) {
				var_down += (adr_down_good[i]-adr)*(adr_down_good[i]-adr);
			}
			var_down /=num_down-1;
		}

		/* compute variance with the median */
		if(num_up>1)
		{
			for (i = 0; i < num_up; ++i) {
				varmed_up += (adr_up_good[i]-percentile_up)*(adr_up_good[i]-percentile_up);
			}
			varmed_up /=num_up-1;
		}
		if(num_down>1)
		{
			for (i = 0; i < num_down; ++i) {
				varmed_down += (adr_down_good[i]-percentile_down)*(adr_down_good[i]-percentile_down);
			}
			varmed_down /=num_down-1;
		}


		if(num_down<6 || num_up<6)
			if ( !quiet)
				printf("WARNING! Too few successful measurements! Mean and variance not significant...\n");

		if ( !quiet)
		{
			printf("\t----- Capacity -----\n");
			printf("avg ADR Downlink: %.2f Mbps; median: %.2f Mbps; variation index: %.2f %%\n", adr, percentile_down, sqrt(var_down)/adr*100.0);
			printf("avg ADR Uplink  : %.2f Mbps; median: %.2f Mbps; variation index: %.2f %%\n", adr_uplink, percentile_up, sqrt(var_up)/adr_uplink*100.0);
			printf("# successful trains (down and up) : %d - %d\n", num_down, num_up);
//			//printf("avg ADR Down: %.2f Mbps; median: %.2f Mbps; var index: %.2f %%; varmed index: %.2f %%\n", adr, percentile_down, var_down/adr*100.0, varmed_down/percentile_down*100.0);
//			//printf("avg ADR Up  : %.2f Mbps; median: %.2f Mbps; var index: %.2f %%; varmed index: %.2f %%\n", adr_uplink, percentile_up, var_up/adr_uplink*100.0, varmed_up/percentile_up*100.0);
//			printf("ADR Down:");
//			for (i = 0; i < NUM_ADR; ++i) {
//				printf(" %.2f", adr_down[i]);
//			}
//			printf("\n#num pkts used:");
//			for (i = 0; i < NUM_ADR; ++i) {
//				printf(" %d", adr_pkts[i]);
//			}
//			printf("\nADR Up:");
//			for (i = 0; i < NUM_ADR; ++i) {
//				printf(" %.2f", adr_up[i]);
//			}
//			printf("\n#num pkts used:");
//			for (i = 0; i < NUM_ADR; ++i) {
//				printf(" %d", adr_pkts_uplink[i]);
//			}
//			printf("\n");
		}
	}
	else
	{
		percentile_up=0;
		percentile_down=0;
		var_down=0;
		var_up=0;
		varmed_down=0;
		varmed_up=0;
		if ( !quiet)
		{
			printf("\t----- Capacity -----\n");
			printf("User-specified ADR Down: %.2f Mbps\n", adr);
		}
	}

	if(has_xtraff)
		if ( !quiet)
			printf("Cross-traffic detected!\n");

	if ( !quiet)
	{
		printf("\nMin/Max RTT: %.2f/%.2f ms\n", min_rtt/1000.0, max_rtt/1000.0);
		printf("Measurements finished at %s \n", buff);
		printf("Measurement latency is %.2f sec \n", time_to_us_delta(
				exp_start_time, exp_end_time)/1000000);
	}
	/* Fields separated by \t: IP timestamp (0/1flags: asymmetry congested_uplink reorderingADR inaccurateADR(I) inaccurateADRup(I) IPID lossyStream reorderedStream) latency(s) ADRdown ADRup(Mbps) down/upRATIO ADRpktsDown ADRpktsUp minAB% maxAB% minAB maxAB(Mbps) minRTT maxRTT(ms) (added by "host" command: DNSlookup) */
/*	printf("IP %s\n", inet_ntoa(*(const struct in_addr *) &dst_ip)); //IP address
	printf("ts %ld\n", exp_end_time.tv_sec); //timestamp
	printf("asymmetry %d\n", asymmetry); //asymmetry (0/1)
	printf("congested_uplink %d\n", congested_uplink); //uplink congestion (0/1)
	printf("reorderingADR %d\n", reordering); //reorderingADRdown (0/1)
	printf("inaccurateADR %d\n", inaccurate); //inaccurateADR (0/1) (I)
	printf("inaccurateADR up %d\n", inaccurate_up); //inaccurateADR uplink (0/1) (I)
	printf("IPIDs %d\n", IPIDs); //IPID (0/1)
	printf("lossy_stream_flag %d\n", lossy_stream_flag); //lossy Streams (0/1)
	printf("reordered_stream %d\n", reordered_stream); //reordered Streams (0/1)
	printf("latency %.2f\n", time_to_us_delta(exp_start_time, exp_end_time)/1000000); //latency(s)
	printf("ADRdown %.2f\n", adr); //ADRdown
	printf("ADRup %.2f\n", adr_uplink); //ADRup
	printf("down/upRATIO %.2f\n", adr/adr_uplink); //down/upRATIO ADR
	printf("adr_pkts %d\n", adr_pkts); //ADRdown train length
	printf("adr_pkts_uplink %d\n", adr_pkts_uplink); //ADRup train length
	printf("min_percent %.0f\n", min_percent); //min avail-bw %
	printf("max_percent %.0f\n", max_percent); //max avail-bw %
	printf("min avail-bw %.2f\n", min); //min avail-bw (Mbps)
	printf("max avail-bw %.2f\n", max); //max avail-bw (Mbps)
	printf("min RTT %.2f\n", min_rtt/1000.0); //min RTT (ms)
	printf("max RTT %.2f\n", max_rtt/1000.0); //max RTT (ms)
*/

	FILE *outfile = fopen("output.log" , "a" ) ;
	/* Fields separated by \t: IP timestamp (0/1flags: asymmetry congested_uplink reorderingADR inaccurateADR(I) inaccurateADRup(I) IPID lossyStream reorderedStream) latency(s) ADRdown ADRup(Mbps) down/upRATIO ADRpktsDown ADRpktsUp minAB% maxAB% minAB maxAB(Mbps) minRTT maxRTT(ms) (added by "host" command: DNSlookup) */
	fprintf(outfile, "%s\t", inet_ntoa(*(const struct in_addr *) &dst_ip)); //IP address
	fprintf(outfile, "%ld\t", exp_end_time.tv_sec); //timestamp
	fprintf(outfile, "%d\t", asymmetry); //asymmetry (0/1)
	fprintf(outfile, "%d\t", congested_uplink); //uplink congestion (0/1)
	fprintf(outfile, "%d\t", reordering); //reorderingADR down (0/1)
	fprintf(outfile, "%d\t", has_xtraff); //xtraff detected on ADR down (0/1)
//	fprintf(outfile, "%d\t", inaccurate); //inaccurateADR (0/1) (I)
//	fprintf(outfile, "%d\t", inaccurate_up); //inaccurateADR uplink (0/1) (I)
	fprintf(outfile, "%d\t", IPIDs); //IPID (0/1)
	fprintf(outfile, "%d\t", lossy_stream_flag); //lossy Streams (0/1)
	fprintf(outfile, "%d\t", reordered_stream); //reordered Streams (0/1)
	fprintf(outfile, "%.2f\t", time_to_us_delta(exp_start_time, exp_end_time)/1000000); //latency(s)
/*	fprintf(outfile, "%.2f\t", adr); //ADRdown
	fprintf(outfile, "%.2f\t", adr_uplink); //ADRup
	fprintf(outfile, "%.2f\t", adr/adr_uplink); //down/upRATIO ADR
	fprintf(outfile, "%d\t", adr_pkts); //ADRdown train length
	fprintf(outfile, "%d\t", adr_pkts_uplink); //ADRup train length
*/	fprintf(outfile, "%.0f\t", min_percent); //min avail-bw %
	fprintf(outfile, "%.0f\t", max_percent); //max avail-bw %
	fprintf(outfile, "%.2f\t", min); //min avail-bw (Mbps)
	fprintf(outfile, "%.2f\t", max); //max avail-bw (Mbps)
	fprintf(outfile, "%.2f\t", min_rtt/1000.0); //min RTT (ms)
	fprintf(outfile, "%.2f\t", max_rtt/1000.0); //max RTT (ms)

	fprintf(outfile, "%d\t", num_down); //ADRdown
	fprintf(outfile, "%.0f\t", adr*1000.0);
	fprintf(outfile, "%.0f\t", percentile_down*1000.0);
	fprintf(outfile, "%.0f\t", sqrt(var_down)/adr*1000.0);
	fprintf(outfile, "%.0f\t", sqrt(varmed_down)/percentile_down*1000.0);
	fprintf(outfile, "%d\t", num_up); //ADRup
	fprintf(outfile, "%.0f\t", adr_uplink*1000.0);
	fprintf(outfile, "%.0f\t", percentile_up*1000.0);
	fprintf(outfile, "%.0f\t", sqrt(var_up)/adr_uplink*1000.0);
	fprintf(outfile, "%.0f", sqrt(varmed_up)/percentile_up*1000.0);
	for (i = 0; i < NUM_ADR; ++i) {
		fprintf(outfile, "\t%.0f", adr_down[i]*1000.0); //ADRdown
	}
	for (i = 0; i < NUM_ADR; ++i) {
		fprintf(outfile, "\t%.0f", adr_up[i]*1000.0); //ADRup
	}
	for (i = 0; i < NUM_ADR; ++i) {
		fprintf(outfile, "\t%d", adr_pkts[i]);
	}
	for (i = 0; i < NUM_ADR; ++i) {
		fprintf(outfile, "\t%d", adr_pkts_uplink[i]);
	}
	fprintf(outfile, "\n");

	fclose(outfile);
	exit(0);
}

double grey_bw_resolution() {
	if (adr)
		return (.05*adr<12?.05*adr:12);
	else
		return min_rate;
}

l_int32 check_intr_coalescence(struct timeval time[], l_int32 len,
		l_int32 *burst) {
	double delta[MAX_STREAM_LEN];
	l_int32 b2b=0, tmp=0;
	l_int32 i;
	l_int32 min_gap;

	min_gap = MIN_TIME_INTERVAL /*> 3*rcv_latency ? MIN_TIME_INTERVAL
			: 3*rcv_latency*/;
	//printf("---%d\n",len);
	for (i=2; i<len; i++) {
		delta[i] = time_to_us_delta(time[i-1], time[i]);
		if (delta[i] <= min_gap) {
			b2b++;
			tmp++;
		} else {
			if (tmp >=3) {
				(*burst)++;
				tmp=0;
			}
		}
	}

	//fprintf(stderr,"\tNumber of b2b %d, Number of burst %d\n",b2b,*burst);
	if (b2b > .6*len) {
		return 1;
	} else
		return 0;
}

/* eliminates packets received b2b due to IC */
l_int32 eliminate_b2b_pkt_ic(double rcvr_time_stamp[], double owd[],
		double owdfortd[], l_int32 low, l_int32 high, l_int32 *num_rcvr_cs,
		l_int32 *tmp_b2b) {
	l_int32 b2b_pkt[MAX_STREAM_LEN];
	l_int32 i, k=0;
	l_int32 len=0;
	l_int32 min_gap;
	l_int32 tmp=0;

	min_gap = MIN_TIME_INTERVAL /*> 3*rcv_latency ? MIN_TIME_INTERVAL
			: 3*rcv_latency*/;
	for (i = low; i <= high; i++) {
		if (rcvr_time_stamp[i] == 0 || rcvr_time_stamp[i+1] == 0)
			continue;

		//fprintf(stderr,"i %d  owd %.2f dispersion %.2f",i, owd[i],rcvr_time_stamp[i+1]- rcvr_time_stamp[i]);
		if ((rcvr_time_stamp[i+1]- rcvr_time_stamp[i])< min_gap) {
			b2b_pkt[k++] = i;
			tmp++;
			//fprintf(stderr," b\n");
		} else {
			if (tmp >= 3) {
				//fprintf(stderr," j\n");
				tmp=0;
				owdfortd[len++] = owd[i];
			}
		}
	}
	return len;
}

l_int32 get_sndr_time_interval(double snd_time[], double *sum) {
	l_int32 k, j=0, new_j=0;
	double ordered[MAX_STREAM_LEN];
	double ltime_interval[MAX_STREAM_LEN];
	for (k = 0; k < stream_len-1; k++) {
		if (snd_time[k] == 0 || snd_time[k+1] == 0)
			continue;
		else
			ltime_interval[j++] = snd_time[k+1] - snd_time[k];
	}
	order_dbl(ltime_interval, ordered, 0, j);
	/* discard the top 15% as outliers  */
	new_j = j - rint(j*.15);
	for (k = 0; k < new_j; k++)
		*sum += ordered[k];
	return new_j;
}

/* Adjust offset to zero again  */
void adjust_offset_to_zero(double owd[], l_int32 len) {
	l_int32 owd_min = 0;
	l_int32 i;
	for (i=0; i< len; i++) {
		if (owd_min == 0 && owd[i] != 0)
			owd_min=owd[i];
		else if (owd_min != 0 && owd[i] != 0 && owd[i]<owd_min)
			owd_min=owd[i];
	}

	for (i=0; i< len; i++) {
		if (owd[i] != 0)
			owd[i] -= owd_min;
	}
}

/*
 splits stream iff sender sent packets more than
 time_interval+1000 usec apart.
 */
l_int32 eliminate_sndr_side_CS(double sndr_time_stamp[], l_int32 split_owd[]) {
	l_int32 j = 0, k=0;
	l_int32 cs_threshold;

	cs_threshold = 2*time_interval>time_interval+1000 ? 2*time_interval
			: time_interval+1000;
	for (k = 0; k < stream_len-1; k++) {
		if (sndr_time_stamp[k] == 0 || sndr_time_stamp[k+1] == 0)
			continue;
		else if ((sndr_time_stamp[k+1]-sndr_time_stamp[k]) > cs_threshold)
			split_owd[j++] = k;
	}
	return j;
}

/*
 discards owd of packets received when
 receiver was NOT running.
 */
l_int32 eliminate_rcvr_side_CS(double rcvr_time_stamp[], double owd[],
		double owdfortd[], l_int32 low, l_int32 high, l_int32 *num_rcvr_cs,
		l_int32 *tmp_b2b) {
	l_int32 b2b_pkt[MAX_STREAM_LEN];
	l_int32 i, k=0;
	l_int32 len=0;
	l_int32 min_gap;

	min_gap = MIN_TIME_INTERVAL /*> 1.5*rcv_latency ? MIN_TIME_INTERVAL
			: 2.5*rcv_latency*/;
	for (i = low; i <= high; i++) {
		if (rcvr_time_stamp[i] == 0 || rcvr_time_stamp[i+1] == 0)
			continue;
		else if ((rcvr_time_stamp[i+1]- rcvr_time_stamp[i])> min_gap)
		{
			owdfortd[len++] = owd[i];
			if(i==high-1)
				owdfortd[len++] = owd[i+1]; //add last packet!
		}
		else
			b2b_pkt[k++] = i;
	}

	/* go through discarded list and count b2b discards as 1 CS instance */
	for (i=1; i<k; i++)
		if (b2b_pkt[i]-b2b_pkt[i-1] != 1)
			(*num_rcvr_cs)++;
	*tmp_b2b += k;
	return len;
}

void print_contextswitch_info(l_int32 num_sndr_cs[], l_int32 num_rcvr_cs[],
		l_int32 discard[], l_int32 stream_cnt) {
	l_int32 j;

int quiet=1;
	if ( !quiet)
		printf("  # of CS @ sndr        :: ");

	for (j=0; j<stream_cnt-1; j++) {
		if ( !quiet)
			printf(":%2d", num_sndr_cs[j]);
	}
	if ( !quiet)
		printf("\n");
	if ( !quiet)
		printf("  # of CS @ rcvr        :: ");
	for (j=0; j<stream_cnt-1; j++) {
		if ( !quiet)
			printf(":%2d", num_rcvr_cs[j]);
	}
	if ( !quiet)
		printf("\n");

	if ( !quiet)
		printf("  # of DS @ rcvr        :: ");
	for (j=0; j<stream_cnt-1; j++) {
		if ( !quiet)
			printf(":%2d", discard[j]);
	}
	if ( !quiet)
		printf("\n");
}

/*
 Compute the time difference in microseconds between two timeval measurements
 */
double time_to_us_delta(struct timeval tv1, struct timeval tv2) {
	double time_us;
	time_us= (double) ((tv2.tv_sec-tv1.tv_sec)*1000000 + (tv2.tv_usec-tv1.tv_usec));
	return time_us;
}

/*
 Order an array of doubles using bubblesort
 */
void order_dbl(double unord_arr[], double ord_arr[], int start, int num_elems) {
	int i, j, k;
	double temp;
	for (i=start, k=0; i<start+num_elems; i++, k++)
		ord_arr[k]=unord_arr[i];
	for (i=1; i<num_elems; i++) {
		for (j=i-1; j>=0; j--)
			if (ord_arr[j+1] < ord_arr[j]) {
				temp=ord_arr[j];
				ord_arr[j]=ord_arr[j+1];
				ord_arr[j+1]=temp;
			} else
				break;
	}
}


void get_sending_rate() {
	time_interval = snd_time_interval/num;
	cur_req_rate = tr;
	cur_actual_rate = cur_pkt_sz * 8. / time_interval;

	if (!equal(cur_req_rate, cur_actual_rate)) {
		if (!grey_max && !grey_min) {
			if (tr_min && tr_max && (less_than(cur_actual_rate,tr_min)||equal(cur_actual_rate,tr_min)))
				converged_rmn_rmx_tm = 1;
			if (tr_min && tr_max && (less_than(tr_max,cur_actual_rate)||equal(tr_max,cur_actual_rate)))
				converged_rmn_rmx_tm = 1;

		} else if (cur_req_rate < tr_max && cur_req_rate > grey_max) {
			if (!(less_than(cur_actual_rate,tr_max)&&grtr_than(cur_actual_rate,grey_max)))
				converged_gmx_rmx_tm = 1;
		} else if (cur_req_rate < grey_min && cur_req_rate > tr_min) {
			if (!(less_than(cur_actual_rate,grey_min) && grtr_than(cur_actual_rate,tr_min)))
				converged_gmn_rmn_tm = 1;
		}
	}

	tr = cur_actual_rate;
	transmission_rate = (l_int32) rint(1000000 * tr);
	if ( !quiet)
		printf("  Fleet Parameter(act)  :: R=%.2fMbps, L=%ldB, K=%ldpackets, T=%ldusec\n",
				cur_actual_rate, cur_pkt_sz, stream_len, time_interval);
	snd_time_interval=0;
	num=0;
}

void get_pct_trend(double pct_metric[], l_int32 pct_trend[],
		l_int32 pct_result_cnt) {
	l_int32 i;
	for (i=0; i < pct_result_cnt; i++) {
		pct_trend[i] = UNCL;
		if (pct_metric[i] == -1) {
			if ( !quiet)
				printf("d");
			pct_trend[i] = DISCARD;
		} else if (pct_metric[i] > 1.1 * PCT_THRESHOLD) {
			if ( !quiet)
				printf("I");
			pct_trend[i] = INCR;
		} else if (pct_metric[i] < .9 * PCT_THRESHOLD) {
			if ( !quiet)
				printf("N");
			pct_trend[i] = NOTR;
		} else if (pct_metric[i] <= PCT_THRESHOLD*1.1 && pct_metric[i] >= PCT_THRESHOLD*.9) {
			if ( !quiet)
				printf("U");
			pct_trend[i] = UNCL;
		}
	}
	if ( !quiet)
		printf("\n");
}

void get_pdt_trend(double pdt_metric[], l_int32 pdt_trend[],
		l_int32 pdt_result_cnt) {
	l_int32 i;
	for (i=0; i < pdt_result_cnt; i++) {
		if (pdt_metric[i] == 2) {
			if ( !quiet)
				printf("d");
			pdt_trend[i] = DISCARD;
		} else if (pdt_metric[i] > 1.1 * PDT_THRESHOLD) {
			if ( !quiet)
				printf("I");
			pdt_trend[i] = INCR;
		} else if (pdt_metric[i] < .9 * PDT_THRESHOLD) {
			if ( !quiet)
				printf("N");
			pdt_trend[i] = NOTR;
		} else if (pdt_metric[i] <= PDT_THRESHOLD*1.1 && pdt_metric[i] >= PDT_THRESHOLD*.9) {
			if ( !quiet)
				printf("U");
			pdt_trend[i] = UNCL;
		}
	}
	if ( !quiet)
		printf("\n");
}

/*
 returns : trend in fleet or -1 if more than 50% of stream were discarded
 */
l_int32 aggregate_trend_result() {
	l_int32 total=0, i_cnt = 0, n_cnt = 0;
	l_int32 num_dscrd_strm=0;
	l_int32 i=0;
	l_int32 pct_trend[TREND_ARRAY_LEN], pdt_trend[TREND_ARRAY_LEN];

	if ( !quiet)
		printf("  PCT metric/stream[%2d] :: ", trend_idx);
	for (i=0; i < trend_idx; i++) {
		if ( !quiet)
			printf("%3.2f:", pct_metric[i]);
	}
	if ( !quiet)
		printf("\n");
	if ( !quiet)
		printf("  PDT metric/stream[%2d] :: ", trend_idx);
	for (i=0; i < trend_idx; i++) {
		if ( !quiet)
			printf("%3.2f:", pdt_metric[i]);
	}
	if ( !quiet)
		printf("\n");
	if ( !quiet)
		printf("  PCT Trend/stream [%2d] :: ", trend_idx);
	get_pct_trend(pct_metric, pct_trend, trend_idx);
	if ( !quiet)
		printf("  PDT Trend/stream [%2d] :: ", trend_idx);
	get_pdt_trend(pdt_metric, pdt_trend, trend_idx);

	if ( !quiet)
		printf("  Trend per stream [%2d] :: ", trend_idx);
	for (i=0; i < trend_idx; i++) {
		if (pct_trend[i] == DISCARD || pdt_trend[i] == DISCARD) {
			if ( !quiet)
				printf("d");
			num_dscrd_strm++;
		} else if (pct_trend[i] == INCR && pdt_trend[i] == INCR) {
			if ( !quiet)
				printf("I");
			i_cnt++;
		} else if (pct_trend[i] == NOTR && pdt_trend[i] == NOTR) {
			if ( !quiet)
				printf("N");
			n_cnt++;
		} else if (pct_trend[i] == INCR && pdt_trend[i] == UNCL) {
			if ( !quiet)
				printf("I");
			i_cnt++;
		} else if (pct_trend[i] == NOTR && pdt_trend[i] == UNCL) {
			if ( !quiet)
				printf("N");
			n_cnt++;
		} else if (pdt_trend[i] == INCR && pct_trend[i] == UNCL) {
			if ( !quiet)
				printf("I");
			i_cnt++;
		} else if (pdt_trend[i] == NOTR && pct_trend[i] == UNCL) {
			if ( !quiet)
				printf("N");
			n_cnt++;
		} else {
			if ( !quiet)
				printf("U");
		}
		total++;
	}
	if ( !quiet)
		printf("\n");

	/* check whether number of usable streams is
	 at least 50% of requested number of streams */
	total-=num_dscrd_strm;
	if (total < num_stream/2 && !slow && !interrupt_coalescence) {
		bad_fleet_cs = 1;
		retry_fleet_cnt_cs++;
		return -1;
	} else {
		bad_fleet_cs = 0;
		retry_fleet_cnt_cs=0;
	}

	if (total==0) {
		if ( !quiet)
			printf("  Aggregate trend       :: UNDETERMINED\n");
		increase_stream_len=1;
		return UNCL;
	} else if ((double)i_cnt/(total) >= AGGREGATE_THRESHOLD) {
		if ( !quiet)
			printf("  Aggregate trend       :: INCREASING\n");
		return INCREASING;
	} else if ((double)n_cnt/(total) >= AGGREGATE_THRESHOLD) {
		if ( !quiet)
			printf("  Aggregate trend       :: NO TREND\n");
		return NOTREND;
	} else {
		if ( !quiet)
			printf("  Aggregate trend       :: GREY\n");
		return GREY;
	}
}

/*
 test if Rmax and Rmin range is smaller than
 user specified bw resolution
 or
 if Gmin and Gmax range is smaller than grey
 bw resolution.
 */
l_int32 converged() {
	int ret_val=0;
	if ((converged_gmx_rmx_tm && converged_gmn_rmn_tm) || converged_rmn_rmx_tm)
		ret_val=1;
	else if (tr_max != 0 && tr_max != tr_min) {
		if (tr_max - tr_min <= bw_resol) {
			converged_rmn_rmx=1;
			ret_val=1;
		} else if (tr_max - grey_max <= grey_bw_resolution()&&grey_min - tr_min <= grey_bw_resolution()) {
			converged_gmn_rmn = 1;
			converged_gmx_rmx = 1;
			ret_val=1;
		}
	}
	return ret_val;
}

/*
 PCT test to detect increasing trend in stream
 */
double pairwise_comparision_test(double array[], l_int32 start, l_int32 end) {
	l_int32 improvement = 0, i;
	double total;

	if (( end - start ) >= MIN_PARTITIONED_STREAM_LEN) {
		for (i = start; i < end - 1; i++) {
			if (array[i] < array[i+1])
				improvement += 1;
		}
		total = ( end - start );
		return ( (double)improvement/total );
	} else
		return -1;
}

/*
 PDT test to detect increasing trend in stream
 */
double pairwise_diff_test(double array[], l_int32 start, l_int32 end) {
	double y = 0, y_abs = 0;
	l_int32 i;
	if (( end - start ) >= MIN_PARTITIONED_STREAM_LEN) {
		for (i = start+1; i < end; i++) {
			y += array[i] - array[i-1];
			y_abs += fabs(array[i] - array[i-1]);
		}
		return y/y_abs;
	} else
		return 2.;
}

l_int32 less_than(double a, double b) {
	if (!equal(a,b) && a < b)
		return 1;
	else
		return 0;
}

l_int32 grtr_than(double a, double b) {
	if (!equal(a,b) && a > b)
		return 1;
	else
		return 0;
}

/*
 if a approx-equal b, return 1
 else 0
 */
l_int32 equal(double a, double b) {
	l_int32 maxdiff;
	if (a<b ? a : b < 500)
		maxdiff = 2.5;
	else
		maxdiff = 5;
	if (fabs(a - b)/ b <= .02 && fabs(a-b)< maxdiff)
		return 1;
	else
		return 0;
}

/*
 Order an array of float using bubblesort
 */
void order_float(float unord_arr[], float ord_arr[], int start, int num_elems) {
	int i, j, k;
	double temp;
	for (i=start, k=0; i<start+num_elems; i++, k++)
		ord_arr[k]=unord_arr[i];
	for (i=1; i<num_elems; i++) {
		for (j=i-1; j>=0; j--)
			if (ord_arr[j+1] < ord_arr[j]) {
				temp=ord_arr[j];
				ord_arr[j]=ord_arr[j+1];
				ord_arr[j+1]=temp;
			} else
				break;
	}
}

/*
 Order an array of int using bubblesort
 */
void order_int(int unord_arr[], int ord_arr[], int num_elems) {
	int i, j;
	int temp;
	for (i=0; i<num_elems; i++)
		ord_arr[i]=unord_arr[i];
	for (i=1; i<num_elems; i++) {
		for (j=i-1; j>=0; j--)
			if (ord_arr[j+1] < ord_arr[j]) {
				temp=ord_arr[j];
				ord_arr[j]=ord_arr[j+1];
				ord_arr[j+1]=temp;
			} else
				break;
	}
}

l_int32 send_latency() {
	char *pack_buf;
	float min_OSdelta[50], ord_min_OSdelta[50];
	int i, len;
	int sock_udp;
	struct timeval first_time, current_time;
	struct sockaddr_in snd_udp_addr, rcv_udp_addr;

	if (max_pkt_sz == 0 || (pack_buf = malloc(max_pkt_sz*sizeof(char)) ) == NULL) {
		printf("ERROR : send_latency : unable to malloc %ld bytes \n",
				max_pkt_sz);
		exit(-1);
	}
	if ((sock_udp=socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("socket(AF_INET,SOCK_DGRAM,0):");
		exit(-1);
	}
	bzero((char*)&snd_udp_addr, sizeof(snd_udp_addr));
	snd_udp_addr.sin_family = AF_INET;
	snd_udp_addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
	snd_udp_addr.sin_port = 0;
	if (bind(sock_udp, (struct sockaddr*)&snd_udp_addr, sizeof(snd_udp_addr))< 0) {
		perror("bind(sock_udp):");
		exit(-1);
	}

	len = sizeof(rcv_udp_addr);
	if (getsockname(sock_udp, (struct sockaddr *)&rcv_udp_addr, &len)< 0) {
		perror("getsockname");
		exit(-1);
	}

	if (connect(sock_udp, (struct sockaddr *)&rcv_udp_addr, sizeof(rcv_udp_addr))< 0) {
		perror("connect(sock_udp)");
		exit(-1);
	}
	srandom(getpid()); /* Create random payload; does it matter? */
	for (i=0; i<max_pkt_sz-1; i++)
		pack_buf[i]=(char)(random()&0x000000ff);
	for (i=0; i<50; i++) {
		gettimeofday(&first_time, NULL);
		if (send(sock_udp, pack_buf, max_pkt_sz, 0)== -1)
			perror("sendto");
		gettimeofday(&current_time, NULL);
		recv(sock_udp, pack_buf, max_pkt_sz, 0);
		min_OSdelta[i] = time_to_us_delta(first_time, current_time);
	}
	/* Use median  of measured latencies to avoid outliers */
	order_float(min_OSdelta, ord_min_OSdelta, 0, 50);
	if (pack_buf != NULL)
		free(pack_buf);
	return (ord_min_OSdelta[25]);
}


#define NUM_SELECT_CALL 31
void min_sleeptime() {
	struct timeval sleep_time, time[NUM_SELECT_CALL];
	int res[NUM_SELECT_CALL], ord_res[NUM_SELECT_CALL];
	int i;
	l_int32 tm;
	gettimeofday(&time[0], NULL);
	for (i=1; i<NUM_SELECT_CALL; i++) {
		sleep_time.tv_sec=0;
		sleep_time.tv_usec=1;
		gettimeofday(&time[i], NULL);
		select(0, NULL, NULL, NULL, &sleep_time);
	}
	for (i=1; i<NUM_SELECT_CALL; i++) {
		res[i-1] = (time[i].tv_sec-time[i-1].tv_sec)*1000000+time[i].tv_usec-time[i-1].tv_usec;
#ifdef DEBUG
		printf("DEBUG :: %.2f ",(time[i].tv_sec-time[i-1].tv_sec)*1000000.+
				time[i].tv_usec-time[i-1].tv_usec);
		printf("DEBUG :: %d \n",res[i-1]);
#endif
	}
	order_int(res, ord_res, NUM_SELECT_CALL-1);
	min_sleep_interval=(ord_res[NUM_SELECT_CALL/2]+ord_res[NUM_SELECT_CALL/2+1])/2;
	gettimeofday(&time[0], NULL);
	tm = min_sleep_interval+min_sleep_interval/4;
	for (i=1; i<NUM_SELECT_CALL; i++) {
		sleep_time.tv_sec=0;
		sleep_time.tv_usec=tm;
		gettimeofday(&time[i], NULL);
		select(0, NULL, NULL, NULL, &sleep_time);
	}
	for (i=1; i<NUM_SELECT_CALL; i++) {
		res[i-1] = (time[i].tv_sec-time[i-1].tv_sec)*1000000+time[i].tv_usec-time[i-1].tv_usec;
#ifdef DEBUG
		printf("DEBUG :: %.2f ",(time[i].tv_sec-time[i-1].tv_sec)*1000000.+
				time[i].tv_usec-time[i-1].tv_usec);
		printf("DEBUG :: %d \n",res[i-1]);
#endif
	}
	order_int(res, ord_res, NUM_SELECT_CALL-1);
	min_timer_intr=(ord_res[NUM_SELECT_CALL/2]+ord_res[NUM_SELECT_CALL/2+1])/2-min_sleep_interval;
#ifdef DEBUG
	printf("DEBUG :: min_sleep_interval %ld\n",min_sleep_interval);
	printf("DEBUG :: min_timer_intr %ld\n",min_timer_intr);
#endif
}

void sig_sigusr1() {
	return;
}
void sig_alrm() {
	terminate_gracefully(exp_start_time);
	exit(0);
}

void help() {
	fprintf(stderr, "usage: fabprobe -d DESTINATION [-a|-c CAPACITY] [-p PORT] [-u UDP_PORT] ");
	fprintf(stderr, "[-s SIZE] [-z] [-0][-1][-2][-3][-4][-5] [-q] [-h|-H]\n");
	fprintf(stderr, "-d        : destination host\n");
	fprintf(stderr, "-a        : measure the available bandwidth (NOT by default!)\n");
	fprintf(stderr, "-c        : do NOT measure the capacity (user specified value)\n");
	fprintf(stderr, "-p        : destination port number\n");
	fprintf(stderr, "-u        : UDP destination port for uplink check on ADR\n");
	fprintf(stderr, "-s        : packet size (in Bytes)\n");
	fprintf(stderr, "-z        : vary src port number\n");
	fprintf(stderr, "-0        : set URG flag on (default is off)\n");
	fprintf(stderr, "-1        : set ACK flag on (default is off)\n");
	fprintf(stderr, "-2        : set PSH flag on (default is off)\n");
	fprintf(stderr, "-3        : set RST flag on (default is off)\n");
	fprintf(stderr, "-4        : set SYN flag on (default is off)\n");
	fprintf(stderr, "-5        : set FIN flag on (default is off)\n");
	fprintf(stderr, "-q        : quite mode\n");
	fprintf(stderr, "-h|-H     : print this help and exit\n");
	fprintf(stderr, "\nFor more help please read the README file!\n");
	exit(0);
}


