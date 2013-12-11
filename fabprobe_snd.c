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


/*
 * "Abandon all hope, you who enter here..."
 * Dante Alighieri, "Divine Commedy", Inferno Canto III, Italy, 1321.
 *
 * This code is a modification of ABwProbe's code, which is a modification of Pathload.
 * Good luck! :)
 */

#define LOCAL
#include "fabprobe_gbls.h"
#include "fabprobe_snd.h"
#include "fabprobe_utils.h"

int main(int argc, char* argv[])
{
	struct hostent *host_rcv;
	struct timeval tv1,tv2;
	l_uint32 snd_time ;
	time_t localtm;
	int latency[30],ord_latency[30];
	int i;
	int c ;
	int errflg=0, dest_addr_set=0;
	int good_fleet = 0;
	struct ifreq ifreq;
	char pcap_errbuf[PCAP_ERRBUF_SIZE];
	struct sigaction sigstruct;
	l_int32 trend, prev_trend = 0;
	l_int32 stream_duration;
	fleet_id=0;

	bw_resol=0;
	interrupt_coalescence=0;
	bad_fleet_cs=0;
	num_stream = 1;
	stream_len = STREAM_LEN;
	num=0;
	snd_time_interval=0;
	exp_flag = 1;
	id = getpid();

	gettimeofday(&exp_start_time, NULL);
	increase_stream_len=0;
	lower_bound=0;
	converged_gmx_rmx = 0;
	converged_gmn_rmn = 0;
	converged_rmn_rmx = 0;
	prev_actual_rate = 0;
	prev_req_rate = 0;
	cur_actual_rate = 0;
	cur_req_rate = 0;

	min_rtt = -1;
	max_rtt = -1;
	for (i = 0; i < NUM_ADR; ++i)
	{
		adr_up[i] = -1;
		adr_down[i] = -1;
		adr_up_good[i] = -1;
		adr_down_good[i] = -1;
		adr_pkts[i] = -1;
		adr_pkts_uplink[i] = -1;
	}
	adr_uplink = 0;
	adr = 0;
	asymmetry = 0;
	reordering = 0;
	inaccurate = 0;
	inaccurate_up = 0;
	IPIDs = 1;
	lossy_stream_flag = 0;
	reordered_stream = 0;
	src_ip = 0;
	num_up=0;
	num_down=0;
	has_xtraff=0;
	check_availbw=0;
	max_pkt_sz = 0;				/* Bytes */
	vary_src_port=0;

	/* default options */
	tcp_dest_port = 80;
	udp_dest_port = 80;
	tcp_src_port  = 50003;		/* WARNING: tied to the pcap filter! (if option -z not used) */
	tcp_ack_num   = 5000;		/* start from this ack number */
	tcp_flags     = 0;			/* no flags */

	extern char *optarg;

	quiet=0;

	while ((c = getopt(argc, argv, "s:zhHqac:d:p:u:012345")) != EOF)
		switch (c)
		{
		case 's':
		{
			max_pkt_sz = atoi(optarg);
			if(max_pkt_sz < 40) {
				fprintf(stderr, "Invalid max packet size '%s'.\n", optarg);
				exit(-1);
			}
			break;
		}
		case 'z':
		{
			vary_src_port=1;
			break;
		}
		case 'H':
		case 'h':
			help() ;
			break ;

		case 'q':
			quiet=1;
			break;

		case 'a':
			check_availbw=1;
			break;

		case 'c':
		{
			check_availbw=1;
			adr = atoi(optarg)/1000.0;
			break;
		}
		case 'p':
		{
			char stmp[5];
			strncpy(stmp, optarg, 5);
			if ((sscanf(stmp, "%d", &tcp_dest_port) != 1)) {
				fprintf(stderr, "Invalid dest port '%s' number.\n", optarg);
				exit(-1);
			}
			//printf("Dest port number: %ld\n", tcp_dest_port);
			break;
		}
		case 'u':
		{
			uplink_check=1;
			char stmp[5];
			strncpy(stmp, optarg, 5);
			if ((sscanf(stmp, "%d", &udp_dest_port) != 1)) {
				fprintf(stderr, "Invalid UDP dest port '%s' number.\n", optarg);
				exit(-1);
			}
			break;
		}
		case 'd':
		{
			struct hostent *host_dst;
			struct sockaddr_in ip_dest_address;
			strcpy(hostname, optarg);
			if ((host_dst = gethostbyname(hostname)) == 0) {
				/* check if the user gave ipaddr */
				if (( ip_dest_address.sin_addr.s_addr = inet_addr(hostname) ) == -1) {
					fprintf(stderr, "%s: unknown host\n", hostname);
					exit(-1);
				}
				memcpy((void *) &dst_ip, &ip_dest_address.sin_addr, ip_dest_address.sin_family);
			}
			else {
				if(host_dst->h_addrtype == AF_INET)
					memcpy((void *) &dst_ip, host_dst->h_addr, 4);
				if(host_dst->h_addrtype == AF_INET6) {
					fprintf(stderr, "Currently not working on IPv6!");
					exit(-1);
				}
			}
			dest_addr_set++;
			break;
		}
		case '0':
			tcp_flags |= TH_URG;
			break;

		case '1':
			tcp_flags |= TH_ACK;
			break;

		case '2':
			tcp_flags |= TH_PUSH;
			break;

		case '3':
			tcp_flags |= TH_RST;
			break;

		case '4':
			tcp_flags |= TH_SYN;
			break;

		case '5':
			tcp_flags |= TH_FIN;
			break;

		case '?':
			errflg++;
		}
	if (errflg || !dest_addr_set)
	{
		fprintf(stderr, "usage: fabprobe -d DESTINATION [-a|-c CAPACITY] [-p PORT] [-u UDP_PORT] ");
		fprintf(stderr, "[-s SIZE] [-z] [-0][-1][-2][-3][-4][-5] [-q] [-h|-H]\n");
		exit(-1);
	}

	min_sleeptime();
	/* gettimeofday latency */
	for(i=0;i<30;i++)
	{
		gettimeofday(&tv1,NULL);
		gettimeofday(&tv2,NULL);
		latency[i]=tv2.tv_sec*1000000+tv2.tv_usec-tv1.tv_sec*1000000-tv1.tv_usec;
	}
	order_int(latency,ord_latency,30);
	gettimeofday_latency = ord_latency[15];
#ifdef DEBUG
	printf("DEBUG :: gettimeofday_latency = %d\n",gettimeofday_latency);
#endif


	/* Control stream: TCP connection */
	if ((sock_tcp=socket(AF_INET, SOCK_STREAM, 0)) < 0)
	{
		perror("socket(AF_INET,SOCK_STREAM,0):");
		exit(-1);
	}

	/* get default interface and associated MTU */
	device = pcap_lookupdev(pcap_errbuf);
	if (device == NULL) {
		fprintf(stderr, "Couldn't find default device: %s\n", pcap_errbuf);
		exit(-1);
	};
	strncpy(ifreq.ifr_name, device, sizeof(ifreq.ifr_name));
	if (ioctl (sock_tcp, SIOCGIFMTU, &ifreq) < 0)
	{
		printf("SIOCGIFMTU(%s): %m\n", ifreq.ifr_name);
		return 0;
	}

	/* set packet sizes */
	if(max_pkt_sz==0)
		max_pkt_sz = ifreq.ifr_mtu;
	else
	{
		if (max_pkt_sz > ifreq.ifr_mtu) {
			fprintf(stderr, "NIC MTU is lower than %d: setting max size to %d\n", max_pkt_sz, ifreq.ifr_mtu);
			max_pkt_sz = ifreq.ifr_mtu;
		}
	}
	min_pkt_size=max_pkt_sz;

	localtm = time(NULL);
	if ( !quiet)
		printf("\n\nMeasurement towards host %s (%s) started on %s\n", hostname, inet_ntoa(*(const struct in_addr *) &dst_ip), ctime(&localtm));
	if ( !quiet )
		printf("  Maximum packet size          :: %ld bytes\n",max_pkt_sz);

	snd_time = (l_int32) send_latency();
	if ( !quiet )
		printf("  send latency @sndr           :: %ld usec\n", snd_time);

	min_time_interval = SCALE_FACTOR * snd_time;
	min_time_interval = min_time_interval>MIN_TIME_INTERVAL ? min_time_interval
			: MIN_TIME_INTERVAL;
	if ( !quiet)
		printf("  Minimum packet spacing       :: %ld usec\n", min_time_interval);

	max_rate = (max_pkt_sz) * 8. / min_time_interval;
	min_rate = (min_pkt_size) * 8./ MAX_TIME_INTERVAL;
	if ( !quiet)
		printf("  Max rate(max_pktsz/min_time) :: %.2fMbps\n", max_rate);

	/***********************************************************************************/
	/*************** DSLprobe starts here **********************************************/
	/***********************************************************************************/

	/* compute ADR (capacity) 10 times*/
	if(adr==0)
	{
		/* UPLINK capacity */
		for (run_num = 0; run_num < NUM_ADR; ++run_num)
		{
			adr_up[run_num] = get_adr(40);
			usleep(250000>2*min_rtt? 250000 : 2*min_rtt); //500ms
			if(adr_up[run_num]!=-1)
			{
				adr_uplink += adr_up[run_num];
				adr_up_good[num_up++] = adr_up[run_num];
			}
		}
		adr_uplink /= num_up;

		/* DOWNLINK capacity */
		for (run_num = 0; run_num < NUM_ADR; ++run_num)
		{
			adr_down[run_num] = get_adr(max_pkt_sz);
			usleep(250000>2*min_rtt? 250000 : 2*min_rtt); //1s

			if(adr_down[run_num]!=-1)
			{
				adr += adr_down[run_num];
				adr_down_good[num_down++] = adr_down[run_num];
			}
		}
		adr /= num_down;
	}
	else
	{
		for (run_num = 0; run_num < NUM_ADR; ++run_num)
		{
			adr_up[run_num] = 0;
			adr_down[run_num] = 0;
		}
		num_up=-1;
		num_down=-1;
		printf("  User specified ADR downlink  :: %.2f\n", adr);
	}

	/***********************************************************************************/
	/*************** DSLprobe ends here ************************************************/
	/***********************************************************************************/

	if (bw_resol == 0 && adr != 0)
		bw_resol = .02*adr;
	else if (bw_resol == 0)
		bw_resol = 2;
//	if ( !quiet)
//		printf("  Grey bandwidth resolution    :: %.2f\n", grey_bw_resolution()); //fake in FAB-Probe

	if (interrupt_coalescence) {
		bw_resol = .05*adr;
		if ( !quiet)
			printf("  Interrupt coalescion detected\n");
	}

	if (adr == 0 || adr > max_rate || adr < min_rate)
		tr = (max_rate+min_rate)/2.;
	else
		tr = adr*0.75; //start at 75%

	if(num_down!=-1)
	{
		if(check_availbw && num_down>5 && adr>0.05 && adr<30 && adr_uplink>0.05 && adr_uplink<10)
			usleep(500000>2*min_rtt? 500000 : 2*min_rtt); //1s
		else
			terminate_gracefully(exp_start_time);
	}

	/************************************/
	/* Estimate the available bandwidth.*/
	transmission_rate = (l_uint32)rint(1000000 * tr);
	max_rate_flag = 0;
	min_rate_flag = 0;

	sigemptyset(&sigstruct.sa_mask);
	sigstruct.sa_handler = sig_alrm;
	sigstruct.sa_flags = 0;

#if defined SA_INTERRUPT
	sigstruct.sa_flags |= SA_INTERRUPT;
#endif
	sigaction(SIGALRM, &sigstruct, NULL);
	/* begin avail-bw estimation */
	while (1) {
		if (calc_param()== -1) {
			terminate_gracefully(exp_start_time);
		}
		if (increase_stream_len)
			stream_len=3*STREAM_LEN;
		else
			stream_len = STREAM_LEN;
		double prev_tr = tr;
		int no_trend_factor = 9;
		stream_duration = stream_len * time_interval;
		i=0;
		good_fleet=0;
		slow=0;
		ignore_reordered=1;
		good_fleet=recv_fleet();

		/* decide how long to wait (up to 9 times RTT) */
		if (pct_metric[0] < 0.7 * PCT_THRESHOLD &&
				pdt_metric[0] < 0.5 * PDT_THRESHOLD)
			no_trend_factor = 1;
		else if (pct_metric[0] < 0.9 * PCT_THRESHOLD &&
				pdt_metric[0] < 0.9 * PDT_THRESHOLD)
			no_trend_factor = 2;
		else
			no_trend_factor = 9;
		usleep(1000000>stream_duration*no_trend_factor? 1000000:stream_duration*no_trend_factor);

		/* process fleet */
		if (good_fleet < 0) {
			if (!increase_stream_len) {
				if(good_fleet== -1) {
					lossy_stream_flag = 1;
				}
				if(good_fleet== -2) {
					reordered_stream = 1;
				}
				trend = INCREASING;
				if (exp_flag == 1 && prev_trend != 0 && prev_trend != trend)
					exp_flag = 0;
				prev_trend = trend;
				if (rate_adjustment(INCREASING)== -1)
					terminate_gracefully(exp_start_time);
			}
		} else {
			get_sending_rate();
			trend = aggregate_trend_result();

			if (trend == -1 && bad_fleet_cs && retry_fleet_cnt_cs >NUM_RETRY_CS)
				terminate_gracefully(exp_start_time);
			else if (( trend == -1 && bad_fleet_cs && retry_fleet_cnt_cs <= NUM_RETRY_CS )) /* repeat fleet with current rate. */
			continue;

			if (trend != GREY) {
				if (exp_flag == 1 && prev_trend != 0 && prev_trend != trend)
					exp_flag = 0;
				prev_trend = trend;
			}

			/* decide rate for next fleet */
			if (rate_adjustment(trend)== -1)
				terminate_gracefully(exp_start_time);
		}
		fleet_id++ ;

		if(trend != GREY && trend !=NOTREND && prev_tr >= adr*0.7)
		{
			/* Test if uplink is congested */
			if ( !quiet)
				printf("\n----Uplink test----");
			congested_uplink=0;

			cur_pkt_sz = 40;
			prev_tr = tr;
			tr = ( cur_pkt_sz ) *8./time_interval;
			good_fleet=recv_fleet();
			if (good_fleet < 0)
			{
				if(good_fleet== -1) {
					lossy_stream_flag = 1;
				}
				if(good_fleet== -2) {
					reordered_stream = 1;
				}
				congested_uplink=1;
				terminate_gracefully(exp_start_time);
			}
			else
			{
				get_sending_rate();
				trend = aggregate_trend_result();

				if (trend == -1 || trend == INCREASING)
				{
					congested_uplink=1;
					terminate_gracefully(exp_start_time);
				}
			}
			if ( !quiet)
				printf("----Uplink not congested----\n");
			tr = prev_tr;
		}
	}
}

