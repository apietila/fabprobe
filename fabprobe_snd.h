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

#ifdef LOCAL
#define EXTERN
#else
#define EXTERN extern
#endif

#ifdef NEW

#define NUM_ADR 10


	#define SCALE_FACTOR                2
	#define NOTREND                     1
	#define INCREASING                  2
	#define GREY                        3

	#define MEDIUM_LOSS_RATE            3
	#define HIGH_LOSS_RATE              15
	#define MIN_PARTITIONED_STREAM_LEN  5       /* number of packet */
	#define MIN_STREAM_LEN              25      /* # of packets */
	#define PCT_THRESHOLD               .55
	#define PDT_THRESHOLD               .4
	#define AGGREGATE_THRESHOLD         .6

	#define NUM_RETRY_RATE_MISMATCH     0
	#define NUM_RETRY_CS                1
	#define MAX_LOSSY_STREAM_FRACTION   50

	#define MIN_TIME_INTERVAL           7         /* microsecond */
	#define MAX_TIME_INTERVAL           200000    /* microsecond */

	/* default snap length (maximum bytes per packet to capture) */
	#define SNAP_LEN 58
	/* ethernet headers are always exactly 14 bytes [1] */
	#define SIZE_ETHERNET 14
	/* Ethernet addresses are 6 bytes */
	#define ETHER_ADDR_LENGTH	6
	/* Ethernet header */
	struct sniff_ethernet {
	        u_char  ether_dhost[ETHER_ADDR_LENGTH];    /* destination host address */
	        u_char  ether_shost[ETHER_ADDR_LENGTH];    /* source host address */
	        u_short ether_type;                     /* IP? ARP? RARP? etc */
	};
	/* IP header */
	struct sniff_ip {
	        u_char  ip_vhl;                 /* version << 4 | header length >> 2 */
	        u_char  ip_tos;                 /* type of service */
	        u_short ip_len;                 /* total length */
	        u_short ip_id;                  /* identification */
	        u_short ip_off;                 /* fragment offset field */
	        #define IP_RF 0x8000            /* reserved fragment flag */
	        #define IP_DF 0x4000            /* dont fragment flag */
	        #define IP_MF 0x2000            /* more fragments flag */
	        #define IP_OFFMASK 0x1fff       /* mask for fragmenting bits */
	        u_char  ip_ttl;                 /* time to live */
	        u_char  ip_p;                   /* protocol */
	        u_short ip_sum;                 /* checksum */
	        struct  in_addr ip_src,ip_dst;  /* source and dest address */
	};
	#define IP_HL(ip)               (((ip)->ip_vhl) & 0x0f)
	#define IP_V(ip)                (((ip)->ip_vhl) >> 4)
	/* TCP header */
	typedef u_int tcp_seq;
	struct sniff_tcp {
	        u_short th_sport;               /* source port */
	        u_short th_dport;               /* destination port */
	        tcp_seq th_seq;                 /* sequence number */
	        tcp_seq th_ack;                 /* acknowledgement number */
	        u_char  th_offx2;               /* data offset, rsvd */
			#define TH_OFF(th)      (((th)->th_offx2 & 0xf0) >> 4)
	        u_char  th_flags;
	        #define TH_FIN  0x01
	        #define TH_SYN  0x02
	        #define TH_RST  0x04
	        #define TH_PUSH 0x08
	        #define TH_ACK  0x10
	        #define TH_URG  0x20
	        #define TH_ECE  0x40
	        #define TH_CWR  0x80
	        #define TH_FLAGS        (TH_FIN|TH_SYN|TH_RST|TH_PUSH|TH_ACK|TH_URG|TH_ECE|TH_CWR)
	        u_short th_win;                 /* window */
	        u_short th_sum;                 /* checksum */
	        u_short th_urp;                 /* urgent pointer */
	};


	EXTERN l_int32 exp_flag;
	EXTERN l_int32 grey_flag;
	EXTERN l_uint32 min_time_interval;
	EXTERN double max_rate , min_rate;
	EXTERN double grey_max , grey_min;
	EXTERN double tr;
	EXTERN double adr;
	EXTERN double tr_max;
	EXTERN double tr_min;
	EXTERN float bw_resol;
	EXTERN l_int32 interrupt_coalescence;
	EXTERN l_int32 max_rate_flag , min_rate_flag;
	EXTERN double cur_actual_rate , cur_req_rate;
	EXTERN double prev_actual_rate , prev_req_rate;
	EXTERN void sig_alrm();
	EXTERN struct timeval exp_start_time;
	/* # of consecutive times act-rate != req-rate */
	EXTERN l_int32 converged_gmx_rmx;
	EXTERN l_int32 converged_gmn_rmn;
	EXTERN l_int32 converged_rmn_rmx;
	EXTERN l_int32 converged_gmx_rmx_tm;
	EXTERN l_int32 converged_gmn_rmn_tm;
	EXTERN l_int32 converged_rmn_rmx_tm;
	EXTERN l_int32 bad_fleet_cs , bad_fleet_rate_mismatch;
	EXTERN l_int32 lower_bound;
	EXTERN l_int32 increase_stream_len;
	EXTERN l_int32 retry_fleet_cnt_cs;
	EXTERN l_int32 num;
	EXTERN l_int32 slow;
	EXTERN double snd_time_interval;
	EXTERN l_int32 ic_flag;
	EXTERN l_int32 exp_fleet_id;
	EXTERN l_int32 repeat_1, repeat_2;
	EXTERN l_int32 trend_idx;
	EXTERN double pct_metric[50], pdt_metric[50];

	EXTERN u_int16_t id;
	EXTERN u_int32_t dst_ip, src_ip;
	EXTERN u_int16_t tcp_dest_port;
	EXTERN u_int16_t udp_dest_port;
	EXTERN u_int16_t tcp_src_port;
	EXTERN u_int32_t tcp_ack_num;
	EXTERN u_int8_t  tcp_flags;
	EXTERN char *device;
	EXTERN char hostname[256];

	EXTERN int ping_host();
	EXTERN int probe_host(int probe_size);
	EXTERN void sig_sigusr1();
//	EXTERN double get_adr();
	EXTERN void	print_payload(const u_char *payload, int len);
	EXTERN void	print_hex_ascii_line(const u_char *payload, int len, int offset);
	EXTERN l_int32 recv_train(l_int32 , struct timeval *, l_int32, int, double*, int* );
	EXTERN void *send_train(void *arg);
	EXTERN void *send_fleet(void *arg);
	EXTERN double grey_bw_resolution();
	EXTERN l_int32 check_intr_coalescence(struct timeval time[],l_int32, l_int32 * );
	EXTERN void terminate_gracefully(struct timeval exp_start_time);
	EXTERN l_int32 rate_adjustment(l_int32 flag);
	EXTERN void get_sending_rate();
	EXTERN l_int32 aggregate_trend_result();
	EXTERN void get_pct_trend(double[] , l_int32[], l_int32 );
	EXTERN void get_pdt_trend(double[] , l_int32[], l_int32 );
	EXTERN int get_trend(double owdfortd[],l_int32 pkt_cnt );
	EXTERN void radj_greymin();
	EXTERN void radj_greymax();
	l_int32 converged();
	EXTERN void radj_notrend();
	EXTERN void radj_increasing();
	EXTERN double get_adr(int pkt_size);
	EXTERN l_int32 get_sndr_time_interval(double snd_time[],double *sum);
	EXTERN void adjust_offset_to_zero(double owd[], l_int32 last_pkt_id);
	EXTERN l_int32 eliminate_sndr_side_CS (double sndr_time_stamp[], l_int32 split_owd[] );
	EXTERN l_int32 eliminate_rcvr_side_CS ( double rcvr_time_stamp[] , double[], double[],l_int32,l_int32,l_int32 *,l_int32 * );
	EXTERN l_int32 eliminate_b2b_pkt_ic ( double rcvr_time_stamp[] , double[], double[],l_int32,l_int32,l_int32 *,l_int32 * );
	EXTERN void print_contextswitch_info(l_int32 num_sndr_cs[], l_int32 num_rcvr_cs[],l_int32 discard[],l_int32 stream_cnt);
	EXTERN double pairwise_comparision_test (double array[] , l_int32 start , l_int32 end);
	EXTERN double pairwise_diff_test(double array[] ,l_int32 start , l_int32 end);
	EXTERN l_int32 equal(double a , double b);
	EXTERN l_int32 less_than(double a , double b);
	EXTERN l_int32 grtr_than(double a , double b);
	EXTERN sem_t sem1, sem2;
	EXTERN int finished_send, resend_fleet, ignore_reordered, congested_uplink;
	EXTERN int vary_src_port, min_pkt_size;

//	#define DEBUG
	#define CHECK_SEQ_NUM

	EXTERN double min_rtt, max_rtt;
	EXTERN double adr_uplink, adr_up[NUM_ADR], adr_down[NUM_ADR], adr_up_good[NUM_ADR], adr_down_good[NUM_ADR];
	EXTERN int asymmetry, reordering, inaccurate, inaccurate_up, IPIDs, adr_pkts[NUM_ADR], adr_pkts_uplink[NUM_ADR];
	EXTERN int lossy_stream_flag, reordered_stream;
	EXTERN int run_num, num_up, num_down, has_xtraff, check_availbw;

#endif

#if defined THRLIB && defined NEW
typedef struct
{
//	pthread_t ptid;
//	l_int32 finished_stream;
//	l_int32 stream_cnt;

	int train_len;
	int train_id;
	struct timeval *time;
	int pkt_size;

} thr_arg;
#endif

//EXTERN int send_fleet();
//EXTERN int send_train();
EXTERN int send_ctr_mesg(char *ctr_buff, l_int32 ctr_code);
EXTERN l_int32 recv_ctr_mesg( char *ctr_buff);
EXTERN l_int32 send_latency();
EXTERN void min_sleeptime();
EXTERN void order_int(int unord_arr[], int ord_arr[], int num_elems);
EXTERN double time_to_us_delta(struct timeval tv1, struct timeval tv2);
EXTERN l_int32 fleet_id_n;
EXTERN l_int32 fleet_id;
EXTERN int sock_udp, sock_tcp, ctr_strm, send_buff_sz, rcv_tcp_adrlen;
EXTERN struct sockaddr_in snd_udp_addr, snd_tcp_addr, rcv_udp_addr, rcv_tcp_addr;
EXTERN l_int32 min_sleep_interval;
/* in usec */
EXTERN l_int32 min_timer_intr;
/* in usec */
EXTERN int gettimeofday_latency;

EXTERN void order_dbl(double unord_arr[], double ord_arr[],int start, int num_elems);
EXTERN void order_float(float unord_arr[], float ord_arr[],int start, int num_elems);
EXTERN void order_int(int unord_arr[], int ord_arr[], int num_elems);
EXTERN void help();
EXTERN int quiet;
EXTERN int uplink_check, udp_fraction;

