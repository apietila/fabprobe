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
#include "fabprobe_utils.h"

/* main function for DSLprobe */
double get_adr(int pkt_size) {
	struct timeval select_tv, arrv_tv[MAX_STREAM_LEN];
	double delta;
	double bw_msr = 0, udp_bw_msr = 0, last_msr = 0;
	double bad_bw_msr[10];
	int num_bad_train=0;
	int first = 1;
	double sum =0;
	l_int32 exp_train_id;
	l_int32 bad_train = 1;
	l_int32 retry = 0;
	l_int32 ctr_code;
	l_int32 ctr_msg_rcvd;
	l_int32 train_len=0, udp_train_len=0;
	l_int32 last=0, i;
	l_int32 spacecnt=18;
	char ctr_buff[8];
	l_int32 num_burst;
	double correction = 0.0;
	int xtraff_gaps = 0;
	int check_asymmetry = 0;

	if ( !quiet)
		printf("  ADR (%dB) [", pkt_size);
	fflush(stdout);

	exp_train_id = 0;
	while (retry < MAX_TRAIN && bad_train) {
		correction = 0.0;
		xtraff_gaps = 0;
		reordering=0;
		for (i=0; i<MAX_STREAM_LEN; i++) {
			arrv_tv[i].tv_sec=0;
			arrv_tv[i].tv_usec=0;
		}
		if (train_len == 25)
			train_len = 10;
		else if(train_len == 10)
			train_len = 5;
		else
			train_len = TRAIN_LEN - exp_train_id*25;
		if(train_len<=0) /*sanity check*/
			break;

		if(pkt_size>40 && train_len>10) /* speedup */
			for (i=run_num-1; i>=0; i--)
			{
				if(adr_pkts[i]>0)
				{
					if(adr_pkts[i]<10)
					{
						train_len = 10;
						break;
					}else if(adr_pkts[i]<25)
					{
						train_len = 25;
						break;
					}
				}
			}

		//printf("%d", train_len);
		if ( !quiet)
			printf(".");
		fflush(stdout);

		spacecnt--;
		ctr_msg_rcvd = 0;
		bad_train = recv_train(exp_train_id, arrv_tv, train_len/**(pkt_size==40? 5:1 )*/, pkt_size, &correction, &xtraff_gaps);
		/* Compute dispersion and bandwidth measurement */
		if (!bad_train) {
			if(udp_train_len==0)
				udp_train_len = train_len;
			num_burst=0;
			interrupt_coalescence=check_intr_coalescence(arrv_tv, train_len,
					&num_burst);
			last=train_len;
			while (!arrv_tv[last].tv_sec)
				--last;
			delta = time_to_us_delta(arrv_tv[1], arrv_tv[last]);
			bw_msr = ((pkt_size) << 3 /* *=8 */) * (last-1) / delta;

			/* UPLINK estimation */
			if(pkt_size==40)
			{
				if(correction==-1)
				{
					double iat[last-1], ordered_iat[last-1];
					double percentile_iat = 0.0;
					double percent = .5;
					double index = (last-2)*percent;
					double reminder = (last-2)*percent - floor((last-2)*percent);
					int j=0;
					correction = 0.0;
					xtraff_gaps = 0;
					if (! quiet){
						printf("WARNING: unable to use IPIDs for uplink cross-traffic detection.\n");
						printf("Correcting ADR based on %d-th percentile IAT:\n", (int)(100.0*percent));
					}
					for(j = 0; j < last-1; ++j)
						iat[j] = time_to_us_delta(arrv_tv[j],arrv_tv[j+1]);
					order_dbl(iat, ordered_iat, 0, last-1);
					/* percent-th quantile with linear interpolation */
					percentile_iat = ordered_iat[(int)index]*(1.0-reminder) + ordered_iat[(int)index+1]*reminder;

					for(j = 0; j < last-1; ++j)
						if(iat[j] > percentile_iat*2.0)
						{
							correction += iat[j];
							xtraff_gaps++;
#ifdef DEBUG
							printf("IAT %d: %.0fus\n", j, iat[j]);
#endif
						}
#ifdef DEBUG
					printf("%d-th percentile: %.0fus, correction: %.2fms, bad samples: %d\n", (int)(100.0*percent), percentile_iat, correction/1000.0, xtraff_gaps);
#endif
				}
				else{
#ifdef DEBUG
					if(correction!=0)
						printf("Proposed correction: %.2fms\n", correction/1000.0);
#endif
				}

#ifdef DEBUG
				if (! quiet)
					if(correction!=0)
						printf("old ADR was %.2fMbps\n\n", bw_msr);
#endif
				if(correction > delta)
#ifdef DEBUG
					printf("DEBUG: excessive correction (train duration: %.2fms)\n", delta/1000.0);
#endif
;
				else
					bw_msr = ((pkt_size) << 3 /* *=8 */) * (last-1-xtraff_gaps) / (delta-correction);

				adr_pkts_uplink[run_num] = train_len;
			}
			/* DOWNLINK estimation */
			else if(adr_uplink != -1)
			{
				/* check if uplink too tight (high asymmetry) */
				double iat[last-1], ordered_iat[last-1];
				double percentile_iat = 0.0;
				double percent = .5; //median!
				double index = (last-2)*percent;
				double reminder = (last-2)*percent - floor((last-2)*percent);
				int j=0;
				for(j = 0; j < last-1; ++j)
					iat[j] = time_to_us_delta(arrv_tv[j],arrv_tv[j+1]);
				order_dbl(iat, ordered_iat, 0, last-1);
				/* percent-th quantile with linear interpolation */
				percentile_iat = ordered_iat[(int)index]*(1.0-reminder) + ordered_iat[(int)index+1]*reminder;

				if(percentile_iat < 1.3*(40*8*2.65/adr_uplink) && ordered_iat[1] > .7*bw_msr)
				{
					if (! quiet)
						printf("WARNING: IAT %d-th percentile too low (high asymmetry?)\n", (int)(100.0*percent));
					//printf("Uplink b2b time: %.0fus, IAT percentile: %.0fus\n", 40*8*2.65/adr_uplink, percentile_iat);
					bad_train=1;
					if(check_asymmetry==1)
						break; /* allow a second try */

					check_asymmetry=1;
				}
				else
				{
					/* check for uplink cross-traffic */
					if(train_len <= 5)
						if( ordered_iat[last-2] > 15.0*(40*8*2.65/adr_uplink) || ordered_iat[last-2] > 3.0*percentile_iat)
						{
							if (! quiet)
								printf("WARNING: max IAT >> IAT %d-th percentile (uplink cross-traffic?)\n", (int)(100.0*percent));
							//printf("Uplink b2b time: %.0fus, IAT percentile: %.0fus, IAT max: %.0fus\n", 40*8*2.65/adr_uplink, percentile_iat, ordered_iat[last-2]);
							bad_train=1;
						}
						else
						{
							uplink_check = 0;
							adr_pkts[run_num] = train_len;
						}
					else
					{
#define LEN 5
						double iat_head[LEN], iat_tail[LEN], ordered_head[LEN], ordered_tail[LEN];
						double percentile_head = 0.0, percentile_tail = 0.0, lowest_iat = 1000000.0;
						double percent = .5;
						double index = (LEN-1)*percent;
						double reminder = (LEN-1)*percent - floor((LEN-1)*percent);
						int j=0;
						for(j = 0; j < LEN; ++j)
						{
							iat_head[j] = time_to_us_delta(arrv_tv[j],arrv_tv[j+1]);
							iat_tail[j] = time_to_us_delta(arrv_tv[last-LEN+j-1],arrv_tv[last-LEN+j]);
						}
						order_dbl(iat_head, ordered_head, 0, LEN);
						order_dbl(iat_tail, ordered_tail, 0, LEN);
						/* percent-th quantile with linear interpolation */
						percentile_head = ordered_head[(int)index]*(1.0-reminder) + ordered_head[(int)index+1]*reminder;
						percentile_tail = ordered_tail[(int)index]*(1.0-reminder) + ordered_tail[(int)index+1]*reminder;

						if(percentile_head < 1.3*(40*8*2.65/adr_uplink) || percentile_tail < 1.3*(40*8*2.65/adr_uplink)
							|| ordered_head[LEN-1] > 15.0*(40*8*2.65/adr_uplink) || ordered_tail[LEN-1] > 15.0*(40*8*2.65/adr_uplink)
							|| ordered_head[LEN-1] > 3.0*percentile_head || ordered_tail[LEN-1] > 3.0*percentile_tail )
						{
							if (! quiet)
								printf("WARNING: head or tail IATs too low or too high (uplink cross-traffic?)\n");
							//printf("Uplink b2b time: %.0fus\nHead %d-th percentile: %.0fus, head max: %.0fus\n", 40*8*2.65/adr_uplink, (int)(100.0*percent), percentile_head, ordered_head[LEN-1]);
							//printf("Tail %d-th percentile: %.0fus, tail max: %.0fus\n", (int)(100.0*percent), percentile_tail, ordered_tail[LEN-1]);
							bad_train=1;
						}
						else
						{
							uplink_check = 0;
							adr_pkts[run_num] = train_len;
						}
					}
				}
			}
		}
		if (bad_train)
		{
			if(bad_train==2) //not responding
				exit(0);

			retry++;
			/* wait for at least 10msec before requesting another train */
			last=train_len;
			while (!arrv_tv[last].tv_sec)
				--last;
			first=1;
			while (!arrv_tv[first].tv_sec)
				++first;
			delta = time_to_us_delta(arrv_tv[first], arrv_tv[last]);
			bad_bw_msr[num_bad_train++] = ((pkt_size) << 3) * (last-first-1) / delta;
			select_tv.tv_sec=0;
			select_tv.tv_usec=500000;
			select(0, NULL, NULL, NULL, &select_tv);
			exp_train_id++;
		}
	}
//	printf("done\n");
//	for (i=0; i<100; i++) {
//		printf("%d. %ld.%ld\n", i, arrv_tv[i].tv_sec, arrv_tv[i].tv_usec);
//	}

	/* Try measuring DOWNlink with interleaved UDP packets (in case of high asymmetry) */
	xtraff_gaps = 0;
	correction = 0.0;
	if(uplink_check && check_asymmetry && pkt_size!=40 && udp_train_len>10)
	{
		struct timeval udp_arrv_tv[MAX_STREAM_LEN];
		udp_fraction = 1;
		last_msr = bw_msr;
		int bad_udp_train=1;

		if (! quiet)
			printf("\nMeasuring ADR with interleaved UDP packets (dst port %hu)...\n\n", udp_dest_port);
		while(udp_fraction > 0)
		{
			select_tv.tv_sec=0;
			select_tv.tv_usec=500000;
			select(0, NULL, NULL, NULL, &select_tv);
			for (i=0; i<305; i++) {
				udp_arrv_tv[i].tv_sec=0;
				udp_arrv_tv[i].tv_usec=0;
			}
			bad_udp_train = recv_train(exp_train_id, udp_arrv_tv, udp_train_len, pkt_size, &correction, &xtraff_gaps);
			if(!bad_udp_train)
			{
				/* Compute dispersion and bandwidth measurement */
				last=udp_train_len/(1+udp_fraction);
				while (!udp_arrv_tv[last].tv_sec)
					--last;
				delta = time_to_us_delta(udp_arrv_tv[1], udp_arrv_tv[last]);
				udp_bw_msr = ((pkt_size) << 3 /* *=8 */) * (last-1)*(udp_fraction+1) / delta;

				/* check for uplink cross-traffic */
				if(udp_train_len > 5)
				{
#define LEN 5
					double iat_head[LEN], iat_tail[LEN], ordered_head[LEN], ordered_tail[LEN];
					double percentile_head = 0.0, percentile_tail = 0.0, lowest_iat = 1000000.0;
					double percent = .5;
					double index = LEN*percent;
					double reminder = LEN*percent - floor(LEN*percent);
					int j=0;
					for(j = 0; j < LEN; ++j)
					{
						iat_head[j] = time_to_us_delta(udp_arrv_tv[j],udp_arrv_tv[j+1]);
						iat_tail[j] = time_to_us_delta(udp_arrv_tv[last-LEN+j-1],udp_arrv_tv[last-LEN+j]);
					}
					order_dbl(iat_head, ordered_head, 0, LEN);
					order_dbl(iat_tail, ordered_tail, 0, LEN);
					/* percent-th quantile with linear interpolation */
					percentile_head = ordered_head[(int)index]*(1.0-reminder) + ordered_head[(int)index+1]*reminder;
					percentile_tail = ordered_tail[(int)index]*(1.0-reminder) + ordered_tail[(int)index+1]*reminder;

					if(adr_uplink != -1)
						if(percentile_head < 1.5*(40*8*2.65/adr_uplink) || percentile_tail < 1.5*(40*8*2.65/adr_uplink) || ordered_head[LEN-1] > 15.0*(40*8*2.65/adr_uplink) || ordered_tail[LEN-1] > 15.0*(40*8*2.65/adr_uplink) || ordered_head[LEN-1] > 3.0*percentile_head || ordered_tail[LEN-1] > 3.0*percentile_tail)
						{
							if (! quiet)
								printf("WARNING: head or tail IATs too low or too high (uplink cross-traffic?)\n");
							//printf("Uplink b2b time: %.0fus\nHead %d-th percentile: %.0fus, head max: %.0fus\n", 40*8*2.65/adr_uplink, (int)(100.0*percent), percentile_head, ordered_head[LEN-1]);
							//printf("Tail %d-th percentile: %.0fus, tail max: %.0fus\n", (int)(100.0*percent), percentile_tail, ordered_tail[LEN-1]);
							bad_udp_train=1;
						}
						else
						{
							uplink_check = 0;
							bad_udp_train = 0;
						}
				}
				else
				{
					double iat[last-1], ordered_iat[last-1];
					double percentile_iat = 0.0;
					double percent = .5;
					double index = last*percent;
					double reminder = last*percent - floor(last*percent);
					int j=0;
					for(j = 0; j < last; ++j)
						iat[j] = time_to_us_delta(udp_arrv_tv[j],udp_arrv_tv[j+1]);
					order_dbl(iat, ordered_iat, 0, last-1);
					/* percent-th quantile with linear interpolation */
					percentile_iat = ordered_iat[(int)index]*(1.0-reminder) + ordered_iat[(int)index+1]*reminder;

					if(adr_uplink != -1)
						if(percentile_iat < 1.5*(40*8*2.65/adr_uplink) || ordered_iat[last-2] > 15.0*(40*8*2.65/adr_uplink) || ordered_iat[last-2] > 3.0*percentile_iat)
						{
							if (! quiet)
								printf("WARNING: IAT %d-th percentile too low (uplink cross-traffic?)\n", (int)(100.0*percent));
							//printf("Uplink b2b time: %.0fus, IAT percentile: %.0fus\n", 40*8*2.65/adr_uplink, percentile_iat);
							bad_udp_train=1;
						}
						else
						{
							uplink_check = 0;
							bad_udp_train=0;
						}

				}

				if(!bad_udp_train)
				{
					if (! quiet)
						printf("\nADR with %d%% UDP packets = %.2fMbps\n", udp_fraction*100/(1+udp_fraction), udp_bw_msr);
					if(udp_bw_msr-last_msr>.3*udp_bw_msr)
					{
						if (! quiet)
							printf("WARNING: uplink might be to tight!\n");
						last_msr = udp_bw_msr;
						asymmetry=1;
						udp_fraction++;
						if(udp_fraction > 2)
						{
							if (! quiet)
								printf("old ADR was %.2fMbps\n", bw_msr);
							bw_msr = udp_bw_msr;
							adr_pkts[run_num] = udp_train_len;
						}
					}
					else
						if(uplink_check == 0 && asymmetry==1)
						{
							udp_fraction = 0;
							if (! quiet)
								printf("old ADR was %.2fMbps\n", bw_msr);
							bw_msr = last_msr;
							adr_pkts[run_num] = udp_train_len;
						}
				}
				else
					udp_fraction++;
			}
			else
				udp_fraction++; //if bad udp train, do NOT send smaller train but reduce the number of TCP packets (but same train length for simplicity)

			if(udp_fraction > 2)
				udp_fraction = 0;
		}
	}

	if (! quiet) {
		i = spacecnt - (pkt_size==40?0:2);
		putchar(']');
		while (--i>0)
			putchar(' ');
		printf(":: ");
	}
	if (!bad_train) {
		if (! quiet)
			printf("%.2fMbps\n", bw_msr);
//		printf("%ld.%ld\n%ld.%ld\n", arrv_tv[1].tv_sec, arrv_tv[1].tv_usec, arrv_tv[last].tv_sec, arrv_tv[last].tv_usec);
	} else {
		for (i=0; i<num_bad_train; i++)
			if (finite(bad_bw_msr[i]))
				sum += bad_bw_msr[i];
		bw_msr = sum/num_bad_train;
		if (! quiet)
			printf("%.2fMbps (I)\n", bw_msr);

		if(pkt_size==40)
			inaccurate_up = 1;
		else
			inaccurate = 1;

		return -1;
	}

	if(pkt_size==40)
	{
		bw_msr *= 2.65;
		if (! quiet)
			printf("adapted to 2 ATM cells: %.2fMbps\n", bw_msr);
	}
	else
		if (! quiet)
			printf("\n");

	return bw_msr;
}



/* Receive a complete packet train */
l_int32 recv_train(l_int32 exp_train_id, struct timeval *arrival_time, l_int32 train_len, int pkt_size, double* correction, int* xtraff_gaps) {

	l_int32 exp_pack_id;
	l_int32 rcvd = 0;
	int finished = 0, timed_out = 0;
	l_int32 bad_train = 0;
	struct timeval send_time[MAX_STREAM_LEN], send_time_reord[MAX_STREAM_LEN];
	u_int16_t exp_ipid = 0;
	int xtraff_pkts = 0;
//	u_int32_t base_tcp_ack_num = 5000;   /* start from this ack number */


	/*
	 * dissect/print packet
	 */
	void got_packet(u_char *args, const struct pcap_pkthdr *header, const u_char *packet)
	{
		/* declare pointers to packet headers */
		const struct sniff_ethernet *ethernet;  /* The ethernet header [1] */
		const struct sniff_ip *ip;              /* The IP header */
		const struct sniff_tcp *tcp;            /* The TCP header */
		const char *payload;                    /* Packet payload */
		int size_ip;
		int size_tcp;
		int size_payload;
//		printf("\nPacket number %d:\n", rcvd+1);
#ifdef DEBUG
		printf("DEBUG :: Pkt %d. Timestamp: %d.%d\n", rcvd+1, header->ts.tv_sec, header->ts.tv_usec);
#endif

		/* define ethernet header */
		ethernet = (struct sniff_ethernet*)(packet);
		/* define/compute ip header offset */
		ip = (struct sniff_ip*)(packet + SIZE_ETHERNET);
		size_ip = IP_HL(ip)*4;
		if (size_ip < 20) {
			//printf("   * Invalid IP header length: %u bytes\n", size_ip);
			return;
		}
		/* define/compute tcp header offset */
		tcp = (struct sniff_tcp*)(packet + SIZE_ETHERNET + size_ip);
		size_tcp = TH_OFF(tcp)*4;
		if (size_tcp < 20) {
			//printf("DEBUG ::    * Invalid TCP header length: %u bytes\n", size_tcp);
			return;
		}
		/* define/compute tcp payload (segment) offset */
		payload = (u_char *)(packet + SIZE_ETHERNET + size_ip + size_tcp);
		/* compute tcp payload (segment) size */
		size_payload = ntohs(ip->ip_len) - (size_ip + size_tcp);
		if( rcvd == 0 ) {
			exp_ipid = ntohs(ip->ip_id);
#ifdef DEBUG
			/* print source and destination IP addresses */
			printf("DEBUG ::        From: %s\n", inet_ntoa(ip->ip_src));
			printf("DEBUG ::          To: %s\n", inet_ntoa(ip->ip_dst));
			printf("DEBUG ::        IPID: %hu\n", ntohs(ip->ip_id));
			printf("DEBUG ::    Src port: %hu\n", ntohs(tcp->th_sport));
			printf("DEBUG ::    Dst port: %hu\n", ntohs(tcp->th_dport));
			printf("DEBUG ::    Seq numb: %u\n", ntohl(tcp->th_seq));
			printf("DEBUG ::    Ack numb: %u\n", ntohl(tcp->th_ack));
			printf("DEBUG ::    Adv Win.: %hu\n", ntohs(tcp->th_win));
			printf("DEBUG ::    Urg poin: %hu\n", ntohs(tcp->th_urp));
			printf("DEBUG ::    TCPflags: 0x%hhX\n", tcp->th_flags);
			printf("DEBUG ::    TCPflags: ");
			u_char flags_cpy = tcp->th_flags, bitMask = 0x80 /* 1000 0000 */;
			int i=0;
			while( i++ < sizeof(u_char)*8 )
			{
				if( flags_cpy & bitMask )
					printf( "1" );
				else
					printf( "0" );

				bitMask >>= 1;
			}
			printf("  (CWR,ECE,URG,ACK,PSH,RST,SYN,FIN)\n");

			printf("DEBUG ::    Payload : %d bytes\n", size_payload);

			if (tcp_ack_num+exp_pack_id != ntohl(tcp->th_seq) ) {
				printf("\nWARNING: expected seq %u but received %u!\n\n", tcp_ack_num+exp_pack_id, ntohl(tcp->th_seq));
			}
#endif
		}
//		if (size_payload > 0) {
//			printf("   Payload (%d bytes):\n", size_payload);
//			print_payload(payload, size_payload);
//		}
		if (rcvd <= train_len)
		{
//			printf("seq # received: %u\n", ntohl(tcp->th_seq));
			int id = ntohl(tcp->th_seq)-tcp_ack_num;
			send_time_reord[rcvd] = send_time[id];
			arrival_time[rcvd] = header->ts;
			if(exp_ipid!=0)
				if(ntohs(ip->ip_id)!=exp_ipid && rcvd!=0 && IPIDs==1)
				{
#ifdef DEBUG
					printf("WARNING: expected IP ID %hu but received %hu\n", exp_ipid, ntohs(ip->ip_id));
#endif
					*xtraff_gaps += 1;
					xtraff_pkts += ntohs(ip->ip_id) - exp_ipid;
					*correction += time_to_us_delta(arrival_time[rcvd-1],arrival_time[rcvd]);
#ifdef DEBUG
					printf("DEBUG :: %d IAT: %.0fus\n", rcvd, time_to_us_delta(arrival_time[rcvd-1],arrival_time[rcvd]));
#endif
					if(ntohs(ip->ip_id) - exp_ipid > 0)
					{
						exp_ipid = ntohs(ip->ip_id)+1;
					}
				}
				else
					exp_ipid++;
			else
				IPIDs = 0;

			if(pkt_size!=40)
			{
				double rtt = time_to_us_delta(send_time[id],arrival_time[rcvd]);
				if(rtt > 0.0) /*sanity check*/
				{
					if(rtt < min_rtt || min_rtt == -1/*not yet initialized*/)
					{
						min_rtt = rtt;
#ifdef DEBUG
						printf("DEBUG :: min_rtt=%.2f rcv=%ld snd=%ld\n", rtt, arrival_time[rcvd].tv_usec, send_time[id].tv_usec);
#endif
					}
					else if(rtt > max_rtt || max_rtt == -1/*not yet initialized*/)
					{
						max_rtt = rtt;
#ifdef DEBUG
						printf("DEBUG :: max_rtt=%.2f rcv=%ld snd=%ld\n", rtt, arrival_time[rcvd].tv_usec, send_time[id].tv_usec);
#endif
					}
				}
			}

#ifdef CHECK_SEQ_NUM
			if(pkt_size!=40)
			{
				if (tcp_ack_num+exp_pack_id != ntohl(tcp->th_seq) )
				{
//					fprintf(stderr,"\nERROR: expected seq %u but received %u!\n\n", tcp_ack_num+exp_pack_id, ntohl(tcp->th_seq));
//					bad_train=1;
					if(udp_fraction==0)
						reordering=1;
				}
				else
				{
					exp_pack_id++;
				}
			}
#endif
			if (rcvd == train_len/(1+udp_fraction)) {
#ifdef DEBUG
				printf("DEBUG :: Captured all packets.\n");
#endif
				finished=1;
			}
		}
		rcvd++;
		return;
	}


	struct timeval start_time, curr_time;
	/* stuff for transmission */
	int train_id_n;
	int pack_id_n;
	l_int32 ctr_code;
	int i, j, prev_rcvd;
	/* stuff for reception */
	struct sigaction sigstruct;
	struct timeval current_time, prev_time;
	struct timeval select_tv;
	fd_set fdset;
	fd_set readset;
	l_int32 ret_val;
	l_int32 pack_id;
	l_int32 train_id;
	char *pack_buf;
	char ctr_buff[8];
#ifdef THRLIB
	thr_arg arg;
	pthread_t tid;
	pthread_attr_t attr;
#endif

	/* declaring libnet and pcap variables */
	pcap_t *handle;				/* packet capture handle */
	char pcap_errbuf[PCAP_ERRBUF_SIZE];		/* error buffer */
	char *str1 = "src ";
	char *str2 = " and tcp port ";
	char portnum[5];
	sprintf(portnum, "%d", tcp_dest_port);
	char *filter_exp = (char *) malloc((strlen(str1) + strlen(hostname) + strlen(str2) + strlen(portnum) + 1)
										* sizeof(char));		/* filter expression [3] */
	struct bpf_program fp;			/* compiled filter program (expression) */
	bpf_u_int32 mask;			/* subnet mask */
	bpf_u_int32 net;			/* ip */
	int num_packets = train_len;			/* number of packets to capture */

	strcpy(filter_exp, str1);
	strcat(filter_exp, hostname);
	strcat(filter_exp, str2);
	strcat(filter_exp, portnum);

	/* pcap stuff for receiving packets */
	/* get network number and mask associated with capture device */
	if (pcap_lookupnet(device, &net, &mask, pcap_errbuf) == -1) {
		fprintf(stderr, "Couldn't get netmask for device %s: %s\n",
				device, pcap_errbuf);
		net = 0;
		mask = 0;
	}
	/* open capture device */
	handle = pcap_open_live(device, SNAP_LEN, 0, 0, pcap_errbuf);
	if (handle == NULL) {
		fprintf(stderr, "Couldn't open device %s: %s\n", device, pcap_errbuf);
		exit(EXIT_FAILURE);
	}
	/* make sure we're capturing on an Ethernet device [2] */
	if (pcap_datalink(handle) != DLT_EN10MB) {
		fprintf(stderr, "%s is not an Ethernet\n", device);
		exit(EXIT_FAILURE);
	}
	if (pcap_setnonblock(handle, 1, pcap_errbuf) == -1) {
		fprintf(stderr, "Couldn't set non blocking pcap: \n", pcap_errbuf);
		exit(EXIT_FAILURE);
	}
	/* compile the filter expression */
	if (pcap_compile(handle, &fp, filter_exp, 0, net) == -1) {
		fprintf(stderr, "Couldn't parse filter %s: %s\n",
				filter_exp, pcap_geterr(handle));
		exit(EXIT_FAILURE);
	}
	/* apply the compiled filter */
	if (pcap_setfilter(handle, &fp) == -1) {
		fprintf(stderr, "Couldn't install filter %s: %s\n",
				filter_exp, pcap_geterr(handle));
		exit(EXIT_FAILURE);
	}

#ifdef DEBUG
	/* print capture info */
	printf("DEBUG :: Device: %s\n", device);
	printf("DEBUG :: Number of packets: %d\n", num_packets);
	printf("DEBUG :: Filter expression: %s\n", filter_exp);
	printf("DEBUG :: run!");
	fflush(stdout);
#endif

	train_id = exp_train_id;
	exp_pack_id=0;
	start_time.tv_sec = 0;

	sigstruct.sa_handler = sig_sigusr1;
	sigemptyset(&sigstruct.sa_mask);
	sigstruct.sa_flags = 0;
#ifdef SA_INTERRUPT
	sigstruct.sa_flags |= SA_INTERRUPT;
#endif
	sigaction(SIGUSR1, &sigstruct, NULL);
#ifdef THRLIB
	finished_send=0;
	timed_out = 0;
	for (i=0; i<305; i++) {
		send_time[i].tv_sec=0;
		send_time[i].tv_usec=0;
	}
	arg.train_len = train_len;
	arg.train_id = train_id;
	arg.time = send_time;
	arg.pkt_size = pkt_size;
	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);
	if (pthread_create(&tid,&attr, send_train, &arg ) != 0 )
	{
		perror("recv_train::pthread_create");
		fprintf(stdout,"Failed to create thread. exiting...\n");
		exit(-1);
	}

	/* main receiving loop */
	while ( !(finished || timed_out)) {
		FD_ZERO(&fdset);
		FD_SET(pcap_fileno(handle), &fdset);
		select_tv.tv_sec  = 0;
		select_tv.tv_usec = 50000; /* 50ms */
		select(FD_SETSIZE, &fdset, NULL, NULL, &select_tv);
		if (pcap_dispatch(handle, num_packets, got_packet, NULL) == -1) {
			fprintf(stderr, "Error during capture: %s\n", pcap_geterr(handle));
			bad_train=1;
			break;
		}
		if (finished_send) {
			if (finished_send == -1) {
				bad_train=1;
				fprintf(stderr, "Bad train, exiting!\n");
				break;
			}
			/* initialize timeout when last packet has been sent */
			if ( start_time.tv_sec == 0 ) {
				gettimeofday(&start_time, NULL);
				gettimeofday(&prev_time, NULL);
				prev_rcvd=0;
			}
			gettimeofday(&curr_time, NULL);
			/* abort if not finished after 3 secs (2*RTT would be better) */
			if (curr_time.tv_sec-start_time.tv_sec > 3 ) {
				timed_out = 1;
			}
			/* time out if more than 250ms since last packet seen (2*RTT would be better) */
			if (rcvd>0 && rcvd==prev_rcvd)
			{
				if (curr_time.tv_usec-prev_time.tv_usec > 250000 ) {
					timed_out = 1;
				}
			}else
			{
				prev_rcvd = rcvd;
				prev_time = curr_time;
			}
		}
	}

#else
	printf("\n\nNot tested without threads!\n\n");
#endif

	fflush(stderr);
	gettimeofday(&arrival_time[rcvd+1], NULL); /* what is it used for?? */
	/* cleanup */
	pcap_freecode(&fp);
	pcap_close(handle);

	/* now do some checks */
	if (rcvd != train_len/(1+udp_fraction)+1) {
#ifdef DEBUG
		printf("DEBUG :: different train lengths: %d, %d", rcvd, train_len);
#endif
		if(rcvd==0)
		{
			printf("\n\nHost %s (%s) is not responding.\n", hostname, inet_ntoa(*(const struct in_addr *) &dst_ip));
			bad_train=2;
		}
		else
			bad_train=1;
	}
	if( !bad_train )
	{
		if(udp_fraction==0)
		{
#ifdef DEBUG
			for (i=0; i<train_len/(1+udp_fraction)+1; i++)
				printf("DEBUG :: RTT %d: %.2fms\n", i, time_to_us_delta(send_time_reord[i],arrival_time[i])/1000.0);
#endif
			if(*xtraff_gaps > 0)
				if(*xtraff_gaps < .9*rcvd)
				{
					if (! quiet)
						printf("WARNING: detected %d X-traffic packets on the uplink\n", xtraff_pkts);
					if(pkt_size>40 && *xtraff_gaps > rcvd/10.0)
						has_xtraff=1;
				}
				else
				{
					if (! quiet)
						printf("WARNING: detected too many X-traffic packets (%d). Security measures??\n", xtraff_pkts);
					*correction = -1;
				}

			if(exp_ipid==0)
				*correction = -1;
		}
	}
	else
		reordering=0;

	sigstruct.sa_handler = SIG_DFL;
	sigemptyset(&sigstruct.sa_mask);
	sigstruct.sa_flags = 0;
	sigaction(SIGUSR1, &sigstruct, NULL);

	return bad_train;
}

/* Send a complete packet train*/
void *send_train(void *arg) {
	int train_len = ((thr_arg *)arg)->train_len;
	int train_id = ((thr_arg *)arg)->train_id;
	struct timeval *send_time = ((thr_arg *)arg)->time;
	int pkt_size = ((thr_arg *)arg)->pkt_size;
	int pack_id;
	int i;
	libnet_t *l = NULL;
	u_int8_t *packet;
	u_int16_t count;
	u_int16_t tcp_win = 0,
			  tcp_urg_ptr = 0;
	u_int32_t tcp_seq_num = 0,
			  packet_size;
	u_int8_t *tcp_payload = NULL, *udp_payload = NULL;
	u_int32_t tcp_payload_len = 0;
	libnet_ptag_t tcp = 0, udp = 0, ip = 0;
	char errbuf[LIBNET_ERRBUF_SIZE];

	/* initialize the libnet library */
	l = libnet_init(LIBNET_RAW4, device, errbuf);
	if (l == NULL)
	{
		fprintf(stderr, "libnet_init() failed: %s", errbuf);
		exit(EXIT_FAILURE);
	}
	if (src_ip == 0)
	{
		src_ip = libnet_get_ipaddr4(l);
		if (src_ip == -1)
		{
			fprintf(stderr, "Can't determine source IP address (%s).\n",
					libnet_geterror(l));
			exit(-1);
		}
	}
	tcp = LIBNET_PTAG_INITIALIZER;
	udp = LIBNET_PTAG_INITIALIZER;
//	tcp_src_port = 50003; /* WARNING: tied with the pcap filter! */
	tcp_win      = 8192;
	tcp_seq_num  = 7000;
	packet_size  = pkt_size;  /* WARNING: will a packet this size reach the destination? */
	tcp_payload_len = packet_size - 40;
	tcp_payload = NULL;
	udp_payload = NULL;

	srandom(getpid()); /* Create random payload; does it matter? */
	if (tcp_payload_len > 0)
	{
		tcp_payload = (u_int8_t *) malloc(tcp_payload_len);
		if (!tcp_payload) {
			fprintf(stderr, "malloc() failed -- exiting\n");
			exit(-1);
		}
		//bzero((char *) tcp_payload, tcp_payload_len);   //set payload to all zeros (clear memory)
		for (i=0; i<tcp_payload_len-1; i++)
			tcp_payload[i]=(random()&0x000000ff);
	}
	if( udp_fraction > 0)
	{
		udp_payload = (u_int8_t *) malloc(tcp_payload_len+13);
		if (!udp_payload) {
			fprintf(stderr, "malloc() failed -- exiting\n");
			exit(-1);
		}
		//bzero((char *) tcp_payload, tcp_payload_len);   //set payload to all zeros (clear memory)
		for (i=0; i<tcp_payload_len+12-1; i++)
			udp_payload[i]=(random()&0x000000ff);
	}


#ifdef THRLIB
	/* send packets */
	for (pack_id=0; pack_id <= train_len/(1+udp_fraction); pack_id++) {
		libnet_clear_packet(l);
		tcp = LIBNET_PTAG_INITIALIZER;
		ip = LIBNET_PTAG_INITIALIZER;

		tcp = libnet_build_tcp(
				vary_src_port? tcp_src_port++:tcp_src_port,	/* source port */
				tcp_dest_port,								/* dest port */
				tcp_seq_num,								/* sequence number */
				tcp_ack_num+pack_id,						/* ack number */
				(u_int8_t) tcp_flags,						/* flags */
				tcp_win,									/* window size */
				0,											/* TCP checksum, 0 = autofill */
				tcp_urg_ptr,								/* urgent pointer */
				LIBNET_TCP_H + tcp_payload_len,				/* total TCP packet length including all subsequent data */
				(u_int8_t *) tcp_payload, tcp_payload_len,	/* optional payload */
				l,											/* libnet context */
				tcp											/* libnet protocol tag, 0 == build new one */
		);
		if (tcp == -1) {
			fprintf(stderr, "Can't build TCP header: %s\n", libnet_geterror(l));
			libnet_clear_packet(l);
			finished_send = -1;
			libnet_destroy(l);
			pthread_exit(NULL);
		}

		ip = libnet_build_ipv4(
				LIBNET_IPV4_H + LIBNET_TCP_H + tcp_payload_len, /* length */
				0,                                    /* TOS */
				++id,                                   /* IP ID */
				1 << 14,     //DF                     /* IP Frag */
				128,                                  /* TTL */
				IPPROTO_TCP,                          /* protocol */
				0,                                    /* checksum */
				src_ip,                               /* source IP */
				dst_ip,                               /* destination IP */
				NULL,                                 /* payload */
				0,                                    /* payload size */
				l,                                    /* libnet context */
				ip);                                   /* ptag */
		if (ip == -1)
		{
			fprintf(stderr, "Can't build IP header: %s\n", libnet_geterror(l));
			finished_send = -1;
			libnet_destroy(l);
			pthread_exit(NULL);
		}

		if (libnet_write(l) == -1)
		{
			fprintf(stderr, "Write error: %s\n", libnet_geterror(l));
			finished_send = -1;
			libnet_destroy(l);
			pthread_exit(NULL);
		}
		gettimeofday(&send_time[pack_id], NULL);

		/* send UDP peckets */
		for(i=0; i<udp_fraction; i++)
		{
			libnet_clear_packet(l);
			udp = LIBNET_PTAG_INITIALIZER;
			ip = LIBNET_PTAG_INITIALIZER;

			udp = libnet_build_udp(
					tcp_src_port,								/* source port */
					udp_dest_port,								/* dest port */
					LIBNET_UDP_H + tcp_payload_len+12,				/* total UDP packet length including all subsequent data */
					0,											/* UDP checksum, 0 = autofill */
					(u_int8_t *) udp_payload, tcp_payload_len+12,	/* optional payload */
					l,											/* libnet context */
					udp											/* libnet protocol tag, 0 == build new one */
			);
			if (udp == -1) {
				fprintf(stderr, "Can't build UDP header: %s\n", libnet_geterror(l));
				libnet_clear_packet(l);
				finished_send = -1;
				libnet_destroy(l);
				pthread_exit(NULL);
			}

			ip = libnet_build_ipv4(
					LIBNET_IPV4_H + LIBNET_UDP_H + tcp_payload_len+12, /* length */
					0,                                    /* TOS */
					++id,                                   /* IP ID */
					1 << 14,     //DF                     /* IP Frag */
					128,                                  /* TTL */
					IPPROTO_UDP,                          /* protocol */
					0,                                    /* checksum */
					src_ip,                               /* source IP */
					dst_ip,                               /* destination IP */
					NULL,                                 /* payload */
					0,                                    /* payload size */
					l,                                    /* libnet context */
					ip);                                   /* ptag */
			if (ip == -1)
			{
				fprintf(stderr, "Can't build IP header: %s\n", libnet_geterror(l));
				finished_send = -1;
				libnet_destroy(l);
				pthread_exit(NULL);
			}

			if (libnet_write(l) == -1)
			{
				fprintf(stderr, "UDP write error: %s\n", libnet_geterror(l));
				finished_send = -1;
				libnet_destroy(l);
				pthread_exit(NULL);
			}
		}
	}

	finished_send = 1;
	libnet_clear_packet(l);
	libnet_destroy(l);
	pthread_exit(NULL);
#endif
	libnet_destroy(l);
	return NULL;
}


/*
 Receive streams .
 After each stream, compute the loss rate.
 Mark a stream "lossy" , if loss rate in
 that stream is more than a threshold.
 */
l_int32 recv_fleet() {

	l_int32 total_pkt_rcvd=0, rcvd = 0;
	l_int32 pkt_id = 0;
	l_int32 exp_pkt_id = 0;
	l_int32 stream_cnt = 0; /* 0->n*/
	int timed_out = 0, j, gap=0;
	l_int32 finished_stream = 0;
	l_int32 pkt_lost = 0;
	l_int32 pkt_reordered = 0;
	struct timeval arrv_tv[MAX_STREAM_LEN], snd_tv[MAX_STREAM_LEN], snd_tv_rcvd[MAX_STREAM_LEN];
	int up_xtraff_pkts = 0;
	int ipid_start = 0;
	//	u_int32_t base_tcp_ack_num = 5000;   /* start from this ack number */

	/*
	 * dissect/print packet
	 */
	void got_packet(u_char *args, const struct pcap_pkthdr *header, const u_char *packet)
	{
		/* declare pointers to packet headers */
		const struct sniff_ethernet *ethernet;  /* The ethernet header [1] */
		const struct sniff_ip *ip;              /* The IP header */
		const struct sniff_tcp *tcp;            /* The TCP header */
		const char *payload;                    /* Packet payload */
		int size_ip;
		int size_tcp;
		int size_payload;
//		printf("\nPacket number %d:\n", rcvd+1);
#ifdef DEBUG
		printf("DEBUG :: Pkt %d. "/*Timestamp: %d.%d\n"*/, rcvd+1/*, header->ts.tv_sec, header->ts.tv_usec*/);
#endif
		/* define ethernet header */
		ethernet = (struct sniff_ethernet*)(packet);
		/* define/compute ip header offset */
		ip = (struct sniff_ip*)(packet + SIZE_ETHERNET);
		size_ip = IP_HL(ip)*4;
		if (size_ip < 20) {
			printf("   * Invalid IP header length: %u bytes\n", size_ip);
			return;
		}
		/* define/compute tcp header offset */
		tcp = (struct sniff_tcp*)(packet + SIZE_ETHERNET + size_ip);
		size_tcp = TH_OFF(tcp)*4;
		if (size_tcp < 20) {
			printf("DEBUG ::    * Invalid TCP header length: %u bytes\n", size_tcp);
			return;
		}
#ifdef DEBUG
		if( rcvd == 0 ) {
			ipid_start = ntohs(ip->ip_id);
			/* print source and destination IP addresses */
			printf("DEBUG ::        From: %s\n", inet_ntoa(ip->ip_src));
			printf("DEBUG ::          To: %s\n", inet_ntoa(ip->ip_dst));
			printf("DEBUG ::       IP ID: %hu\n", ntohs(ip->ip_id));
			printf("DEBUG ::    Src port: %hu\n", ntohs(tcp->th_sport));
			printf("DEBUG ::    Dst port: %hu\n", ntohs(tcp->th_dport));
			printf("DEBUG ::    Seq numb: %u\n", ntohl(tcp->th_seq));
			printf("DEBUG ::    Ack numb: %u\n", ntohl(tcp->th_ack));
			printf("DEBUG ::    Adv Win.: %hu\n", ntohs(tcp->th_win));
			printf("DEBUG ::    Urg poin: %hu\n", ntohs(tcp->th_urp));
			printf("DEBUG ::    TCPflags: 0x%hhX\n", tcp->th_flags);
			printf("DEBUG ::    TCPflags: ");
			u_char flags_cpy = tcp->th_flags, bitMask = 0x80 /* 1000 0000 */;
			int i=0;
			while( i++ < sizeof(u_char)*8 )
			{
				if( flags_cpy & bitMask )
					printf( "1" );
				else
					printf( "0" );

				bitMask >>= 1;
			}
			printf("  (CWR,ECE,URG,ACK,PSH,RST,SYN,FIN)\n");

			/* define/compute tcp payload (segment) offset */
			payload = (u_char *)(packet + SIZE_ETHERNET + size_ip + size_tcp);

			/* compute tcp payload (segment) size */
			size_payload = ntohs(ip->ip_len) - (size_ip + size_tcp);

			printf("DEBUG ::    Payload : %d bytes\n", size_payload);

			if (tcp_ack_num+exp_pkt_id != ntohl(tcp->th_seq) ) {
				printf("\nWARNING: expected seq %u but received %u!\n\n", tcp_ack_num+exp_pkt_id, ntohl(tcp->th_seq));
			}
		}
		else if(ipid_start!=0 && ntohs(ip->ip_id)!=ipid_start+rcvd+up_xtraff_pkts)
		{
			printf("WARNING: expected IP ID %hu but received %hu\n", ipid_start+rcvd+up_xtraff_pkts, ntohs(ip->ip_id));
			if(ntohs(ip->ip_id)>ipid_start+rcvd+up_xtraff_pkts)
				up_xtraff_pkts += ntohs(ip->ip_id) - (ipid_start+rcvd+up_xtraff_pkts);
		}
#endif

		if (rcvd <= stream_len) {
#ifdef CHECK_SEQ_NUM
			if ( ntohl(tcp->th_seq) >= tcp_ack_num+exp_pkt_id ) {  /* pkt_id >= exp_pkt_id */
#endif
				arrv_tv[rcvd] = header->ts;
				snd_tv_rcvd[rcvd] = snd_tv[ntohl(tcp->th_seq)-tcp_ack_num];

				double rtt = time_to_us_delta(snd_tv_rcvd[rcvd],arrv_tv[rcvd]);
				if(rtt > 0.0) /*sanity check*/
				{
					if(rtt < min_rtt || min_rtt == -1/*not yet initialized*/)
					{
						min_rtt = rtt;
#ifdef DEBUG
						printf("DEBUG :: min_rtt=%.2f rcv=%ld snd=%ld\n", rtt, arrv_tv[rcvd].tv_usec, snd_tv[rcvd].tv_usec);
#endif
					}
					else if(rtt > max_rtt || max_rtt == -1/*not yet initialized*/)
					{
						max_rtt = rtt;
#ifdef DEBUG
						printf("DEBUG :: max_rtt=%.2f rcv=%ld snd=%ld\n", rtt, arrv_tv[rcvd].tv_usec, snd_tv[rcvd].tv_usec);
#endif
					}
				}
#ifdef CHECK_SEQ_NUM
				if ( ntohl(tcp->th_seq) > tcp_ack_num+exp_pkt_id ) {
					gap = ntohl(tcp->th_seq) - (tcp_ack_num+exp_pkt_id);  /* (pkt_id - exp_pkt_id) */
					pkt_lost += gap;
					exp_pkt_id += gap;  /* +1 is done later */
				}
				exp_pkt_id++;
				rcvd++;

				if ( ntohl(tcp->th_seq) == tcp_ack_num+stream_cnt*stream_len+(stream_len-1) ) {
					finished_stream=1;
				}
			}
			else {
				pkt_reordered++;
			}
#endif
			if (rcvd == stream_len) {
#ifdef DEBUG
				fprintf(stderr, "DEBUG :: Captured all packets.\n");
#endif
				finished_stream=1;
			}
		}
#ifndef CHECK_SEQ_NUM
		exp_pkt_id++;
		rcvd++;
#endif
		return;
	}

	struct sigaction sigstruct;
	struct timeval current_time/*, first_time*/;
	double pkt_loss_rate;
	double owd[MAX_STREAM_LEN];
	double snd_tm[MAX_STREAM_LEN];
	double arrv_tm[MAX_STREAM_LEN];
	l_int32 ctr_code;
	l_int32 stream_id_n, stream_id=0;
	l_int32 lossy_stream = 0;
	l_int32 return_val = 1;
	l_int32 stream_duration;
	l_int32 num_sndr_cs[20], num_rcvr_cs[20];
	double owdfortd[MAX_STREAM_LEN];
	l_int32 num_substream, substream[MAX_STREAM_LEN];
	l_int32 low, high, len;
	l_int32 b2b_pkt_per_stream[20];
	l_int32 tmp_b2b;
#ifdef THRLIB
	pthread_t tid = 0;
	thr_arg arg;
	pthread_attr_t attr;
#endif
	l_int32 num_bursts;
	l_int32 abort_fleet=0;
	l_int32 p=0;
	struct timeval select_tv;
	fd_set readset;
	l_int32 ret_val;
	struct timeval sleep_time;
	double t1 = 0, t2 = 0;
	l_int32 sleep_tm_usec;
	fd_set fdset;

	pcap_t *handle;				/* packet capture handle */
	char pcap_errbuf[PCAP_ERRBUF_SIZE];		/* error buffer */
	char *str1 = "src ";
	char *str2 = " and tcp port ";
	char portnum[5];
	sprintf(portnum, "%d", tcp_dest_port);
	char *filter_exp = (char *) malloc((strlen(str1) + strlen(hostname) + strlen(str2) + strlen(portnum) + 1)
										* sizeof(char));		/* filter expression [3] */
	struct bpf_program fp;			/* compiled filter program (expression) */
	bpf_u_int32 mask;			/* subnet mask */
	bpf_u_int32 net;			/* ip */
	int num_packets = stream_len;			/* number of packets to capture */
	strcpy(filter_exp, str1);
	strcat(filter_exp, hostname);
	strcat(filter_exp, str2);
	strcat(filter_exp, portnum);
	struct timeval start_time, curr_time;
	l_int32 pct[] = {DISCARD,DISCARD,DISCARD,DISCARD,DISCARD,DISCARD,DISCARD,DISCARD,DISCARD},
			pdt[] = {DISCARD,DISCARD,DISCARD,DISCARD,DISCARD,DISCARD,DISCARD,DISCARD,DISCARD},
			overall[]={DISCARD,DISCARD,DISCARD,DISCARD,DISCARD,DISCARD,DISCARD,DISCARD,DISCARD};

	/* pcap stuff for receiving packets */
	/* get network number and mask associated with capture device */
	if (pcap_lookupnet(device, &net, &mask, pcap_errbuf) == -1) {
		fprintf(stderr, "Couldn't get netmask for device %s: %s\n",
				device, pcap_errbuf);
		net = 0;
		mask = 0;
	}
	/* open capture device */
	handle = pcap_open_live(device, SNAP_LEN, 0, 0, pcap_errbuf);
	if (handle == NULL) {
		fprintf(stderr, "Couldn't open device %s: %s\n", device, pcap_errbuf);
		exit(EXIT_FAILURE);
	}
	/* make sure we're capturing on an Ethernet device [2] */
	if (pcap_datalink(handle) != DLT_EN10MB) {
		fprintf(stderr, "%s is not an Ethernet\n", device);
		exit(EXIT_FAILURE);
	}
	if (pcap_setnonblock(handle, 1, pcap_errbuf) == -1) {
		fprintf(stderr, "Couldn't set non blocking pcap: \n", pcap_errbuf);
		exit(EXIT_FAILURE);
	}
	/* compile the filter expression */
	if (pcap_compile(handle, &fp, filter_exp, 0, net) == -1) {
		fprintf(stderr, "Couldn't parse filter %s: %s\n",
				filter_exp, pcap_geterr(handle));
		exit(EXIT_FAILURE);
	}
	/* apply the compiled filter */
	if (pcap_setfilter(handle, &fp) == -1) {
		fprintf(stderr, "Couldn't install filter %s: %s\n",
				filter_exp, pcap_geterr(handle));
		exit(EXIT_FAILURE);
	}
#ifdef DEBUG
	/* print capture info */
	printf("DEBUG :: Device: %s\n", device);
	printf("DEBUG :: Number of packets: %d\n", num_packets);
	printf("DEBUG :: Filter expression: %s\n", filter_exp);
#endif

	trend_idx=0;
	ic_flag = 0;
	if ( !quiet) {
		printf("\nReceiving Fleet %ld\n", exp_fleet_id);
		printf("  Fleet Parameter(req)  :: R=%.2fMbps, L=%ldB, K=%ldpackets, \
T=%ldusec\n",
				tr, cur_pkt_sz, stream_len, time_interval);
	}
	if ( !quiet)
		printf("  Lossrate per stream   :: ");

	sigstruct.sa_handler = sig_sigusr1;
	sigemptyset(&sigstruct.sa_mask);
	sigstruct.sa_flags = 0;
#ifdef SA_INTERRUPT
	sigstruct.sa_flags |= SA_INTERRUPT;
#endif
	sigaction(SIGUSR1, &sigstruct, NULL);
#ifdef THRLIB
	/* to sync with sender */
	sem_init(&sem1,0,0);
	sem_init(&sem2,0,0);

	finished_send=0;
	resend_fleet=0;
	arg.time = snd_tv;

	pthread_attr_init(&attr);
	pthread_attr_setdetachstate(&attr,PTHREAD_CREATE_DETACHED);
	if (pthread_create(&tid,&attr, send_fleet, &arg ) != 0 )
	{
		perror("recv_fleet::pthread_create");
		fprintf(stdout,"Failed to create thread. exiting...\n");
		exit(-1);
	}
#endif

	/* main loop */
	while (stream_cnt < num_stream) {
		pkt_lost = 0;
		start_time.tv_sec = 0;
		for (j=0; j < stream_len; j++) {
			arrv_tv[j].tv_sec=0;
			arrv_tv[j].tv_usec=0;
			snd_tv_rcvd[j].tv_sec=0;
			snd_tv_rcvd[j].tv_usec=0;
		}
		exp_pkt_id = stream_cnt*stream_len;
		rcvd = 0;
		pkt_reordered = 0;
		timed_out = 0;

		/* Receive K packets of nth stream */
#ifdef THRLIB
		while( !(finished_stream|| timed_out))
		{
			FD_ZERO(&fdset);
			FD_SET(pcap_fileno(handle), &fdset);
			select_tv.tv_sec  = 0;
			select_tv.tv_usec = 50000; /* 50ms */
			select(FD_SETSIZE, &fdset, NULL, NULL, &select_tv);

			if (pcap_dispatch(handle, num_packets, got_packet, NULL) == -1) {
				fprintf(stderr, "Error during capture: %s\n", pcap_geterr(handle));
				return_val = -1;
				break;
			}
			if (finished_send) {
				if (finished_send == -1) {
					return_val = -1;
					fprintf(stderr, "Bad fleet!\n");
					break;
				}
				/* initialize timeout when last packet has been sent */
				if ( start_time.tv_sec == 0 ) {
					gettimeofday(&start_time, NULL);
				}
				gettimeofday(&curr_time, NULL);
				/* abort if not finished after 3 secs (2*RTT would be better) */
				if (curr_time.tv_sec-start_time.tv_sec > 3 ) {
					if(rcvd < 10) /* do not consider stream if less than 5 pkts are received */
					{
						resend_fleet = 1;
						start_time.tv_sec = 0;
						finished_send = 0;
						if ( !quiet)
							fprintf(stderr, "\nRepeating entire fleet (CRC computation error?)\n\n");
						sem_post(&sem2);
						pthread_cancel(tid);
						if(rcvd==0)
							exit(-1);
						return 0;
					}
					else
					{
						timed_out = 1;
#ifdef DEBUG
						printf("\nDEBUG :: Timed out (last packet).\n");
#endif
					}
				}
			}
//			usleep(min_sleep_interval>10000?min_sleep_interval:10000);
		}
#else
		printf("\n\nWARNING: Not implemented without threads!\n\n");
#endif

		/* start post process */
		if(pkt_reordered)
			if ( !quiet)
				fprintf(stderr, "\nStream %d: %d reordered packets", stream_cnt, pkt_reordered);
		if(pkt_reordered > .7*HIGH_LOSS_RATE && !(ignore_reordered)) /* do not consider stream if more than .7*HIGH_LOSS_RATE pkts are reordered */
		{
			resend_fleet = 1;
			start_time.tv_sec = 0;
			finished_send = 0;
			if ( !quiet)
				fprintf(stderr, "\nRepeating entire fleet (%d reordered packets)\n\n",pkt_reordered);
			sem_post(&sem2);
			pthread_cancel(tid);
			return -2;
		}

		for (j=0; j < stream_len; j++) {
			snd_tm[j]= snd_tv_rcvd[j].tv_sec * 1000000.0 + snd_tv_rcvd[j].tv_usec;
			arrv_tm[j] = arrv_tv[j].tv_sec * 1000000.0 + arrv_tv[j].tv_usec;
			owd[j] = arrv_tm[j] - snd_tm[j];
//			printf("owd[%d]=%.0f\n", j, owd[j]);
//			printf("%d: owd=%.0f snd_tm=%.0f arrv_tm=%.0f\n", j, owd[j], snd_tm[j], arrv_tm[j]);
		}

		total_pkt_rcvd += rcvd;
		finished_stream = 0;
		exp_pkt_id -= stream_cnt*stream_len;
		pkt_lost += stream_len - exp_pkt_id;
		pkt_loss_rate = (double )pkt_lost * 100. / stream_len;
		if ( !quiet)
			printf(":%.1f", pkt_loss_rate);
		stream_cnt++;
		num_bursts=0;
		if (interrupt_coalescence)
			ic_flag=check_intr_coalescence(arrv_tv, rcvd, &num_bursts);
		if (pkt_loss_rate < HIGH_LOSS_RATE && pkt_loss_rate >= MEDIUM_LOSS_RATE)
			lossy_stream++;
		if (pkt_loss_rate >= HIGH_LOSS_RATE || ( stream_cnt >= num_stream
				&& lossy_stream*100./stream_cnt >= MAX_LOSSY_STREAM_FRACTION )) {
			if (increase_stream_len) {
				increase_stream_len=0;
				lower_bound=1;
			}
			if ( !quiet)
				printf("\n  Fleet aborted due to high lossrate");
			abort_fleet=1;
			break;
		} else {
			/* analyze trend in stream */
			num += get_sndr_time_interval(snd_tm, &snd_time_interval);
			adjust_offset_to_zero(owd, stream_len);
			num_substream = eliminate_sndr_side_CS(snd_tm, substream);
			num_sndr_cs[stream_cnt-1] = num_substream;
			substream[num_substream++]=stream_len-1;
			low=0;
			num_rcvr_cs[stream_cnt-1]=0;
			tmp_b2b=0;
			for (j=0; j<num_substream; j++) {
				high=substream[j];
				if (ic_flag) {
					if (num_bursts < 2) {
						if (++repeat_1 == 3) {
							repeat_1=0;
							/* Abort fleet and try to find lower bound */
							if ( !quiet)
								printf("Warning: num_bursts < 2\n!\n");
							abort_fleet=1;
							lower_bound=1;
							increase_stream_len=0;
							break;
						}
					} else if (num_bursts <= 5) {
						if (++repeat_2 == 3) {
							repeat_2=0;
							/* Abort fleet and retry with longer stream length */
							if ( !quiet)
								printf("Warning: num_bursts <= 5!\n");
							abort_fleet=1;
							increase_stream_len=1;
							break;
						}
					} else {
						/* DOES NOT use descending RTT to eliminate cross-traffic. TO be implemented*/
						if ( !quiet)
							printf("Warning: because of IC, not filtering cross-traffic!\n");
						increase_stream_len=0;
						len=eliminate_b2b_pkt_ic(arrv_tm, owd, owdfortd, low,
								high, &num_rcvr_cs[stream_cnt-1], &tmp_b2b);
							pct_metric[trend_idx]=pairwise_comparision_test(
								owdfortd, 0, len);
						pdt_metric[trend_idx]=pairwise_diff_test(owdfortd, 0,
								len);
						trend_idx+=1;
					}
				} else {
					len=eliminate_rcvr_side_CS(arrv_tm, owd, owdfortd, low,
							high, &num_rcvr_cs[stream_cnt-1], &tmp_b2b);
					if (len > MIN_STREAM_LEN) {
						if(len > 3*MIN_STREAM_LEN && get_trend(owdfortd, len) < MIN_STREAM_LEN)
						{
							if ( !quiet)
								printf("Too many samples eliminated! (Too much cross traffic).\n");
							increase_stream_len=1;
							if(cur_pkt_sz==40)
								return -1;
#ifdef THRLIB
							pthread_cancel(tid);
#endif
							break;
						}
					}
#ifdef DEBUG
					else
						printf("len= %d\n", len);
#endif
				}
				low=high+1;
			}
			if (abort_fleet)
				break;
			else {
				b2b_pkt_per_stream[stream_cnt-1] = tmp_b2b;

				if(trend_idx<=9)
				{
					if (pct_metric[trend_idx-1] == -1) {
						pct[trend_idx-1] = DISCARD;
					} else if (pct_metric[trend_idx-1] > 1.1 *PCT_THRESHOLD) {
						pct[trend_idx-1] = INCR;
					} else if (pct_metric[trend_idx-1] < .9 * PCT_THRESHOLD) {
						pct[trend_idx-1] = NOTR;
					} else if (pct_metric[trend_idx-1] <= PCT_THRESHOLD*1.1 && pct_metric[trend_idx-1] >= PCT_THRESHOLD*.9) {
						pct[trend_idx-1] = UNCL;
					}
					if (pdt_metric[trend_idx-1] == 2) {
						pdt[trend_idx-1] = DISCARD;
					} else if (pdt_metric[trend_idx-1] <= PDT_THRESHOLD*1.1 && pdt_metric[trend_idx-1] >= PDT_THRESHOLD*.9) {
						pdt[trend_idx-1] = UNCL;
					} else if (pdt_metric[trend_idx-1] > 1.1 *PDT_THRESHOLD) {
						pdt[trend_idx-1] = INCR;
					} else if (pdt_metric[trend_idx-1] < .9 * PDT_THRESHOLD) {
						pdt[trend_idx-1] = NOTR;
					}
					if (pct[trend_idx-1] == DISCARD || pdt[trend_idx-1] == DISCARD) {
						overall[trend_idx-1] = DISCARD;
					} else if (pct[trend_idx-1] == INCR && pdt[trend_idx-1] == INCR) {
						overall[trend_idx-1] = INCR;
					} else if (pct[trend_idx-1] == NOTR && pdt[trend_idx-1] == NOTR) {
						overall[trend_idx-1] = NOTR;
					} else if (pct[trend_idx-1] == INCR && pdt[trend_idx-1] == UNCL) {
						overall[trend_idx-1] = INCR;
					} else if (pct[trend_idx-1] == NOTR && pdt[trend_idx-1] == UNCL) {
						overall[trend_idx-1] = NOTR;
					} else if (pdt[trend_idx-1] == INCR && pct[trend_idx-1] == UNCL) {
						overall[trend_idx-1] = INCR;
					} else if (pdt[trend_idx-1] == NOTR && pct[trend_idx-1] == UNCL) {
						overall[trend_idx-1] = NOTR;
					} else {
						overall[trend_idx-1] = UNCL;
					}

					if(trend_idx==1)
					{
						if(pct_metric[0] < 0.7 * PCT_THRESHOLD &&
							pdt_metric[0] < 0.4 * PDT_THRESHOLD)
								slow=1; //take result as good as if the link was slow
						if(pct_metric[0] > 1.3 * PCT_THRESHOLD &&
							pdt_metric[0] > 1.6 * PDT_THRESHOLD)
								slow=1; //take result as good as if the link was slow
					}
					else if(trend_idx==3 || trend_idx==5 || trend_idx==9)
					{
						int i, notr_cnt=0, incr_cnt=0;

						for(i=0; i<trend_idx; i++)
						{
							if(overall[i]==NOTR)
								notr_cnt++;
							if(overall[i]==INCR)
								incr_cnt++;
						}

						switch(trend_idx)
						{
						case 3:
						{
							if(notr_cnt==3 || incr_cnt==3)
								slow=1; //take result as good as if the link was slow
							break;
						}
						case 5:
						{
							if(notr_cnt==4 || incr_cnt==4)
								slow=1; //take result as good as if the link was slow
							break;
						}
						case 9:
						{
							if(notr_cnt==7 || incr_cnt==7)
								slow=1; //take result as good as if the link was slow
							break;
						}
						}
					}
					if(slow==1)
					{
#ifdef THRLIB
						pthread_cancel(tid);
#endif
						break;
					}
				}

#ifdef THRLIB
				/* wait for sender to finish*/
				sem_wait(&sem1);
				finished_send=0;

				/* discard late packets */
				if ( !timed_out && rcvd < stream_len) {
//					printf("\n\n\nwait for reordered packets!\n");
					/* inter-stream latency is max (RTT,9*stream_duration):
					 * wait 2*stream_duration and discard all packets received meanwhile
					 */
					gettimeofday(&start_time, NULL);
					t1 = (double) start_time.tv_sec * 1000000.0 +(double)start_time.tv_usec;
					stream_duration = stream_len * time_interval;
					sleep_tm_usec = stream_duration * 2;
					/* release cpu if gap is longer than min_sleep_time
					 */
					sleep_tm_usec -= min_sleep_interval;
					if (sleep_tm_usec > min_sleep_interval) {
						sleep_time.tv_sec = (int)(sleep_tm_usec / 1000000);
						sleep_time.tv_usec = sleep_tm_usec - sleep_time.tv_sec*1000000;
						select(1, NULL, NULL, NULL, &sleep_time);
					}
					sleep_tm_usec += min_sleep_interval;
					/* busy wait for the remaining time */
					do {
						gettimeofday(&curr_time, NULL);
						t2 = (double) curr_time.tv_sec * 1000000.0 +(double)curr_time.tv_usec;
					} while ((t2 - t1) < sleep_tm_usec);

					/* discard all packets received meanwhile */
					void do_noop(u_char *args, const struct pcap_pkthdr *header, const u_char *packet)
					{
//						printf("\n\npacket arrived after the last one!\n\n");
					}
					while (pcap_dispatch(handle, -1, do_noop, NULL) > 0) ;
				}

				/* done */
				sem_post(&sem2);
#endif
			}
		}

		/* STOP: A hack for slow links */
		stream_duration = stream_len * time_interval;
		if ( stream_duration >= 750000 || (stream_duration >= 500000 && stream_cnt > 4) ) {
			slow=1;
#ifdef THRLIB
			pthread_cancel(tid);
#endif
			break;
		}
	} /*end of while (stream_cnt < num_stream ). */

	if ( !quiet)
		printf("\n");

	if (abort_fleet) {
		if ( !quiet)
			printf("\tAborting fleet. Stream_cnt %d\n", stream_cnt);
#ifdef THRLIB
		pthread_cancel(tid);
#endif
		return_val = -1;
	} else
		print_contextswitch_info(num_sndr_cs, num_rcvr_cs, b2b_pkt_per_stream,
				stream_cnt);

	/* cleanup */
	pcap_freecode(&fp);
	pcap_close(handle);
	sem_destroy(&sem1);
	sem_destroy(&sem2);
	exp_fleet_id++;
	return return_val;
}

/*
 Send fleet for avail-bw estimation.
 */
void *send_fleet(void *arg) {
	int train_len = ((thr_arg *)arg)->train_len;
	int train_id = ((thr_arg *)arg)->train_id;
	struct timeval *send_time = ((thr_arg *)arg)->time;
	int pack_id;
	double time_offset =-3, time_error =0;
	double smoothing_factor =.4;

	libnet_t *l = NULL;
	u_int8_t *packet;
	u_int16_t tcp_win = 0,
			  tcp_urg_ptr = 0;
	u_int32_t tcp_seq_num = 0,
			  packet_size;
	u_int8_t *tcp_payload = NULL;
	u_int32_t tcp_payload_len = 0;
	libnet_ptag_t tcp = 0, ip = 0;
	char errbuf[LIBNET_ERRBUF_SIZE];
	tcp = LIBNET_PTAG_INITIALIZER;
	ip = LIBNET_PTAG_INITIALIZER;
	tcp_win      = 8192;
	tcp_seq_num  = 7000;
	packet_size  = cur_pkt_sz;  /* WARNING: will a packet this size reach the destination? */
	tcp_payload_len = packet_size - 40;
	tcp_payload = NULL;

	if (tcp_payload_len > 0)
	{
		tcp_payload = (u_int8_t *) malloc(tcp_payload_len);
		if (!tcp_payload) {
			fprintf(stderr, "malloc() failed -- exiting\n");
			exit(-1);
		}
		//bzero((char *) tcp_payload, tcp_payload_len);   //set payload to all zeros (clear memory)
		srandom(getpid()); /* Create random payload; does it matter? */
		int i;
		for (i=0; i<tcp_payload_len-1; i++)
			tcp_payload[i]=(random()&0x000000ff);

	}
	struct timeval tmp1, tmp2;
	struct timeval sleep_time;
	double t1=0, t2 = 0;
	l_int32 ctr_code;
	l_int32 pkt_id;
	l_int32 pkt_cnt = 0;
	l_int32 stream_cnt = 0;
	l_int32 stream_id_n = 0;
	l_int32 usec_n, sec_n, pkt_id_n;
	l_int32 sleep_tm_usec;
	int stream_duration;
	int diff;
	int i;
	int no_trend_factor = 9;
	l_int32 tmp=0;
	pkt_id = 0;
#ifdef DEBUG
	fprintf(stderr, "DEBUG :: Sending fleet %ld ", fleet_id);
#endif
	while (stream_cnt < num_stream) {

		for (i=0; i < stream_len; i++) {
			send_time[i].tv_sec=0;
			send_time[i].tv_usec=0;
		}
		if (l != NULL)
		{
			libnet_destroy(l);
			l = NULL;
		}
		/* initialize the libnet library */
		l = libnet_init(LIBNET_RAW4, device, errbuf);
		if (l == NULL)
		{
			fprintf(stderr, "libnet_init() failed: %s", errbuf);
			exit(EXIT_FAILURE);
		}
		if (src_ip == 0)
		{
			src_ip = libnet_get_ipaddr4(l);
			if (src_ip == -1)
			{
				fprintf(stderr, "Can't determine source IP address (%s).\n",
						libnet_geterror(l));
				exit(-1);
			}
		}
		pkt_cnt = 0;
#ifdef DEBUG
		fprintf(stderr, " #");
#endif

#ifdef THRLIB
		if( libnet_build_tcp(
				vary_src_port? tcp_src_port++:tcp_src_port,	/* source port */
				tcp_dest_port,								/* dest port */
				tcp_seq_num,								/* sequence number */
				tcp_ack_num+pkt_cnt+stream_cnt*stream_len,						/* ack number */
				(u_int8_t) tcp_flags,						/* flags */
				tcp_win,									/* window size */
				0,											/* TCP checksum, 0 = autofill */
				tcp_urg_ptr,								/* urgent pointer */
				LIBNET_TCP_H + tcp_payload_len,				/* total TCP packet length including all subsequent data */
				(u_int8_t *) tcp_payload, tcp_payload_len,	/* optional payload */
				l,											/* libnet context */
				tcp											/* libnet protocol tag, 0 == build new one */
				)

			== -1) {
				fprintf(stderr, "Can't build TCP header: %s\n", libnet_geterror(l));
				libnet_clear_packet(l);
				finished_send = -1;
				libnet_destroy(l);
				pthread_exit(NULL);
		}

		if( libnet_build_ipv4(
				LIBNET_IPV4_H + LIBNET_TCP_H + tcp_payload_len, /* length */
				0,                                    /* TOS */
				++id,                                   /* IP ID */
				1 << 14,     //DF                     /* IP Frag */
				128,                                  /* TTL */
				IPPROTO_TCP,                          /* protocol */
				0,                                    /* checksum */
				src_ip,                               /* source IP */
				dst_ip,                               /* destination IP */
				NULL,                                 /* payload */
				0,                                    /* payload size */
				l,                                    /* libnet context */
				ip)                                   /* ptag */

			== -1) {
				fprintf(stderr, "Can't build IP header: %s\n", libnet_geterror(l));
				finished_send = -1;
				libnet_destroy(l);
				pthread_exit(NULL);
		}

		/* loop for sending packets */
		for (pkt_cnt=0; pkt_cnt < stream_len; pkt_cnt++) {
			if (libnet_write(l) == -1)
			{
				fprintf(stderr, "Write error: %s\n", libnet_geterror(l));
				finished_send = -1;
				libnet_destroy(l);
				pthread_exit(NULL);
			}
			gettimeofday(&tmp1, NULL);
			t1 = (double) tmp1.tv_sec * 1000000.0 +(double)tmp1.tv_usec;
			send_time[pkt_cnt] = tmp1;

			libnet_clear_packet(l);

			if( libnet_build_tcp(
					vary_src_port? tcp_src_port++:tcp_src_port,	/* source port */
					tcp_dest_port,								/* dest port */
					tcp_seq_num,								/* sequence number */
					tcp_ack_num+pkt_cnt+1+stream_cnt*stream_len,/* ack number */
					(u_int8_t) tcp_flags,						/* flags */
					tcp_win,									/* window size */
					0,											/* TCP checksum, 0 = autofill */
					tcp_urg_ptr,								/* urgent pointer */
					LIBNET_TCP_H + tcp_payload_len,				/* total TCP packet length including all subsequent data */
					(u_int8_t *) tcp_payload, tcp_payload_len,	/* optional payload */
					l,											/* libnet context */
					0											/* libnet protocol tag, 0 == build new one */
					)
				== -1) {
					fprintf(stderr, "Can't build TCP header: %s\n", libnet_geterror(l));
					libnet_clear_packet(l);
					finished_send = -1;
					libnet_destroy(l);
					pthread_exit(NULL);
			}

			if( libnet_build_ipv4(
					LIBNET_IPV4_H + LIBNET_TCP_H + tcp_payload_len, /* length */
					0,                                    /* TOS */
					++id,                                   /* IP ID */
					1 << 14,     //DF                     /* IP Frag */
					128,                                  /* TTL */
					IPPROTO_TCP,                          /* protocol */
					0,                                    /* checksum */
					src_ip,                               /* source IP */
					dst_ip,                               /* destination IP */
					NULL,                                 /* payload */
					0,                                    /* payload size */
					l,                                    /* libnet context */
					0)                                   /* ptag, 0 == build new one */

				== -1) {
					fprintf(stderr, "Can't build IP header: %s\n", libnet_geterror(l));
					finished_send = -1;
					libnet_destroy(l);
					pthread_exit(NULL);
			}
			if (pkt_cnt) {
				/* compute error correction for time interval (correct up to 50% time_interval)*/
				time_error = (double) time_interval - time_to_us_delta(send_time[pkt_cnt-1],send_time[pkt_cnt]);
				if ( fabs(time_error) < .5*time_interval || fabs(time_offset) > .3*time_interval ) {
					if(time_error)
						time_offset += copysign(log(1.0+pow(time_error+1,2.0)/10.0), time_error);
					else
						/* make it tend towards nearest integer */
						time_offset = smoothing_factor*rint(time_offset) + (1.0-smoothing_factor)*time_offset;
				}
			}
			/* wait */
			gettimeofday(&tmp2, NULL);
			t2 = (double) tmp2.tv_sec * 1000000.0 +(double)tmp2.tv_usec -rint(time_offset) /*error correction*/;
			tmp = (l_int32) (t2-t1);
			if (pkt_cnt < ( stream_len - 1 )) {
				l_int32 tm_remaining = time_interval - tmp;
				if (tm_remaining > 2*min_sleep_interval) {
					sleep_tm_usec = tm_remaining - (tm_remaining%min_timer_intr)-min_timer_intr<200 ? 2*min_timer_intr
							: min_timer_intr;
					sleep_time.tv_sec = (int)(sleep_tm_usec / 1000000);
					sleep_time.tv_usec = sleep_tm_usec - sleep_time.tv_sec*1000000;
					select(1, NULL, NULL, NULL, &sleep_time);
				}
				gettimeofday(&tmp2, NULL);
				t2 = (double) tmp2.tv_sec * 1000000.0 +(double)tmp2.tv_usec -rint(time_offset) /*error correction*/;
				diff = gettimeofday_latency>0 ? gettimeofday_latency-1 : 0;
				while ((t2 - t1) < (time_interval-diff+rint(time_offset) /*error correction*/)) {
					gettimeofday(&tmp2, NULL);
					t2 = (double) tmp2.tv_sec * 1000000.0 +(double)tmp2.tv_usec;
				}
				tmp1 = tmp2;
				t1 = t2;
			}
		}

		finished_send = stream_cnt+1;

		/*done, notify receiver*/
		sem_post( &sem1 );
		/*wait for receiever to finish*/
		sem_wait( &sem2 );

		if(resend_fleet) {
			libnet_destroy(l);
			pthread_exit(NULL);
		}
		stream_cnt++;
		gettimeofday(&tmp2, NULL);
		t2 = (double) tmp2.tv_sec * 1000000.0 +(double)tmp2.tv_usec;
		finished_send=0;
		/* destroy the libnet library and initialize it again (avoids bug on TCP checksum?)*/
		libnet_destroy(l);
		l = NULL;
#endif

		/* A hack for slow links */
		if ( stream_duration >= 750000 || (stream_duration >= 500000 && stream_cnt > 4) ) {
#ifdef THRLIB
			finished_send = 1;
			libnet_destroy(l);
			pthread_exit(NULL);
#endif
			break;
		}
	}
#ifdef THRLIB
	finished_send = 1;
	libnet_destroy(l);
	pthread_exit(NULL);
#endif

	return NULL;
}
