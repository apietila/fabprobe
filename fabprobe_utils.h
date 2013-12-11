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

#ifndef FABPROBE_UTILS_H_
#define FABPROBE_UTILS_H_


/*
 * print data in rows of 16 bytes: offset   hex   ascii
 *
 * 00000   47 45 54 20 2f 20 48 54  54 50 2f 31 2e 31 0d 0a   GET / HTTP/1.1..
 */
void print_hex_ascii_line(const u_char *payload, int len, int offset);

/*
 * print packet payload data (avoid printing binary data)
 */
void print_payload(const u_char *payload, int len);

void sig_sigusr1();
void sig_alrm();

double grey_bw_resolution();

/*
 * send a 40 bytes probe
 *
 */
int ping_host();

/*
 * send a probe of size probe_size (in bytes).
 *
 * returns the size of the probe (in bytes)
 *
 */
int probe_host(int probe_size) ;

l_int32 check_intr_coalescence(struct timeval time[], l_int32 len, l_int32 *burst);

/* eliminates packets received b2b due to IC */
l_int32 eliminate_b2b_pkt_ic(double rcvr_time_stamp[], double owd[],
		double owdfortd[], l_int32 low, l_int32 high, l_int32 *num_rcvr_cs,
		l_int32 *tmp_b2b);

l_int32 get_sndr_time_interval(double snd_time[], double *sum);

/* Adjust offset to zero again  */
void adjust_offset_to_zero(double owd[], l_int32 len);

/*
 splits stream iff sender sent packets more than
 time_interval+1000 usec apart.
 */
l_int32 eliminate_sndr_side_CS(double sndr_time_stamp[], l_int32 split_owd[]);

/*
 discards owd of packets received when
 receiver was NOT running.
 */
l_int32 eliminate_rcvr_side_CS(double rcvr_time_stamp[], double owd[],
		double owdfortd[], l_int32 low, l_int32 high, l_int32 *num_rcvr_cs,
		l_int32 *tmp_b2b);

void print_contextswitch_info(l_int32 num_sndr_cs[], l_int32 num_rcvr_cs[],
		l_int32 discard[], l_int32 stream_cnt);

/*
 Compute the time difference in microseconds between two timeval measurements
 */
double time_to_us_delta(struct timeval tv1, struct timeval tv2);

/*
 Order an array of doubles using bubblesort
 */
void order_dbl(double unord_arr[], double ord_arr[], int start, int num_elems);

/*
 dpending upon trend in fleet :-
 - update the state variables.
 - decide the next fleet rate
 return -1 when converged
 */
l_int32 rate_adjustment(l_int32 flag);

void get_sending_rate();

#define INCR    1
#define NOTR    2
#define DISCARD 3
#define UNCL    4
void get_pct_trend(double pct_metric[], l_int32 pct_trend[],
		l_int32 pct_result_cnt);

void get_pdt_trend(double pdt_metric[], l_int32 pdt_trend[],
		l_int32 pdt_result_cnt);

int get_trend(double owdfortd[], l_int32 pkt_cnt);

/*
 returns : trend in fleet or -1 if more than 50% of stream were discarded
 */
l_int32 aggregate_trend_result();

/*
 test if Rmax and Rmin range is smaller than
 user specified bw resolution
 or
 if Gmin and Gmax range is smaller than grey
 bw resolution.
 */
l_int32 converged();

/*
 PCT test to detect increasing trend in stream
 */
double pairwise_comparision_test(double array[], l_int32 start, l_int32 end);

/*
 PDT test to detect increasing trend in stream
 */
double pairwise_diff_test(double array[], l_int32 start, l_int32 end);

/*
 Calculate next fleet rate
 when fleet showed NOTREND trend.
 */
void radj_notrend();

/*
 Calculate next fleet rate
 when fleet showed INCREASING trend.
 */
void radj_increasing();

/*
 calculates fleet param L,T .
 calc_param returns -1, if we have
 reached to upper/lower limits of the
 stream parameters like L,T .
 otherwise returns 0 .
 */
l_int32 calc_param();

/*
 Calculate next fleet rate
 when fleet showed GREY trend.
 */
void radj_greymax();

/*
 Calculate next fleet rate
 when fleet showed GREY trend.
 */
void radj_greymin();

void terminate_gracefully(struct timeval exp_start_time);

l_int32 less_than(double a, double b);

l_int32 grtr_than(double a, double b);

/*
 if a approx-equal b, return 1
 else 0
 */
l_int32 equal(double a, double b);

/*
 Order an array of float using bubblesort
 */
void order_float(float unord_arr[], float ord_arr[], int start, int num_elems);

/*
 Order an array of int using bubblesort
 */
void order_int(int unord_arr[], int ord_arr[], int num_elems);

l_int32 send_latency();

#define NUM_SELECT_CALL 31
void min_sleeptime();

void help();

#endif /*FABPROBE_UTILS_H_*/
