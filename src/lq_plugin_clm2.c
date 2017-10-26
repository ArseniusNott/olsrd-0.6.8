
/*
 * The olsr.org Optimized Link-State Routing daemon(olsrd)
 * Copyright (c) 2008 Henning Rogge <rogge@fgan.de>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of olsr.org, olsrd nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Visit http://www.olsr.org for more information.
 *
 * If you find this software useful feel free to make a donation
 * to the project. For more information see the website or contact
 * the copyright holders.
 *
 */

#include "tc_set.h"
#include "link_set.h"
#include "lq_plugin.h"
#include "olsr_spf.h"
#include "lq_packet.h"
#include "packet.h"
#include "olsr.h"
#include "lq_plugin_clm2.h"
#include "parser.h"
#include "fpm.h"
#include "mid_set.h"
#include "scheduler.h"
#include "log.h"
#include <time.h>

/*
	Modified: Arsenius
	UNDER THE `NIHANETS PROJECT 3
	DEVELOPMENT OF ROUTING METRICS FOR HIGH THROUGHPUT
	AND CONGESTION FREE ROUTING OVER WIRELESS COMMUNITY MESH NETWORKS

	DATE: August 10, 2017
	PURPOSE: header file to allow to print uint8_t
*/
#include <inttypes.h>
/*
	Modification ends here
*/

/*
	Modified: Arsenius
	UNDER THE BAYANIHANETS PROJECT 3
	DEVELOPMENT OF ROUTING METRICS FOR HIGH THROUGHPUT
	AND CONGESTION FREE ROUTING OVER WIRELESS COMMUNITY MESH NETWORKS

	DATE: August 14, 2017
	PURPOSE: header files to add CLM2_PRESENTER TO THE METRIC PLUGIN
	EDIT THIS LATER
*/
#include <sys/socket.h>
#include <linux/netlink.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
/*
	Modification ends here
*/

/*
	Modified: Arsenius
	UNDER THE BAYANIHANETS PROJECT 3
	DEVELOPMENT OF ROUTING METRICS FOR HIGH THROUGHPUT
	AND CONGESTION FREE ROUTING OVER WIRELESS COMMUNITY MESH NETWORKS

	DATE: August 14, 2017
	PURPOSE: Data structures needed to interface kernel space to userspace
*/
#define NETLINK_USER 31

#define MAX_PAYLOAD 1024 /* maximum payload size*/
struct sockaddr_nl src_addr2, dest_addr2;
struct nlmsghdr *nlh2 = NULL;
struct iovec iov2;
int sock_fd2;
struct msghdr msg2;

//For CLM2 Statistics
static unsigned long long int clm2_measurement_time;
static unsigned long long int clm2_busy_time;
static unsigned long long int clm2_nav_read_counter;
static unsigned long long int clm2_nav_busy_counter;

//For handling the requested clm2 and clm2_nav values to be used by serialize_lq_tc
static uint8_t clm2_cca_value = 0;
static uint8_t clm2_nav_value = 0;

//highest value between true_cca and true_nav values
static uint8_t true_lq_value = 0;
/*
	Modification ends here
*/

static void default_lq_initialize_clm2(void);

static olsr_linkcost default_lq_calc_cost_clm2(const void *lq);

static void default_lq_packet_loss_worker_clm2(struct link_entry *link, void *lq, bool lost);
static void default_lq_memorize_foreign_hello_clm2(void *local, void *foreign);

static int default_lq_serialize_hello_lq_pair_clm2(unsigned char *buff, void *lq);
static void default_lq_deserialize_hello_lq_pair_clm2(const uint8_t ** curr, void *lq);
static int default_lq_serialize_tc_lq_pair_clm2(unsigned char *buff, void *lq);
static void default_lq_deserialize_tc_lq_pair_clm2(const uint8_t ** curr, void *lq);

static void default_lq_copy_link2neigh_clm2(void *t, void *s);
static void default_lq_copy_link2tc_clm2(void *target, void *source);
static void default_lq_clear_clm2(void *target);
static void default_lq_clear_clm2_hello(void *target);

static const char *default_lq_print_clm2(void *ptr, char separator, struct lqtextbuffer *buffer);
static const char *default_lq_print_cost_clm2(olsr_linkcost cost, struct lqtextbuffer *buffer);

/*
	Modified: Arsenius
	UNDER THE BAYANIHANETS PROJECT 3
	DEVELOPMENT OF ROUTING METRICS FOR HIGH THROUGHPUT
	AND CONGESTION FREE ROUTING OVER WIRELESS COMMUNITY MESH NETWORKS

	DATE: August 14, 2017
	PURPOSE: add dummy function declaration to return a static value
	This is useful to augment clm2_presenter later when this is successfully implemented
*/
static void get_clm2_stats(void);
/*
	Modification ends here
*/

/* etx lq plugin (freifunk fpm version) settings */
struct lq_handler lq_clm2_handler = {
  &default_lq_initialize_clm2,
  &default_lq_calc_cost_clm2,
  &default_lq_calc_cost_clm2,

  &default_lq_packet_loss_worker_clm2,

  &default_lq_memorize_foreign_hello_clm2,
  &default_lq_copy_link2neigh_clm2,
  &default_lq_copy_link2tc_clm2,
  &default_lq_clear_clm2_hello,
  &default_lq_clear_clm2,

  &default_lq_serialize_hello_lq_pair_clm2,
  &default_lq_serialize_tc_lq_pair_clm2,
  &default_lq_deserialize_hello_lq_pair_clm2,
  &default_lq_deserialize_tc_lq_pair_clm2,

  &default_lq_print_clm2,
  &default_lq_print_clm2,
  &default_lq_print_cost_clm2,

  sizeof(struct default_lq_clm2_hello),
  sizeof(struct default_lq_clm2),
  4,
  4
};

static void
default_lq_clm2_handle_lqchange(void) {
  struct default_lq_clm2_hello *lq;
  struct ipaddr_str buf;
  struct link_entry *link;

  bool triggered = false;

  OLSR_FOR_ALL_LINK_ENTRIES(link) {
    bool relevant = false;
    lq = (struct default_lq_clm2_hello *)link->linkquality;

    if (lq->smoothed_lq.valueLq < lq->lq.valueLq) {
      if (lq->lq.valueLq == 255 || lq->lq.valueLq - lq->smoothed_lq.valueLq > lq->smoothed_lq.valueLq/10) {
        relevant = true;
      }
    }
    else if (lq->smoothed_lq.valueLq > lq->lq.valueLq) {
      if (lq->smoothed_lq.valueLq - lq->lq.valueLq > lq->smoothed_lq.valueLq/10) {
        relevant = true;
      }
    }
    if (lq->smoothed_lq.valueNlq < lq->lq.valueNlq) {
      if (lq->lq.valueNlq == 255 || lq->lq.valueNlq - lq->smoothed_lq.valueNlq > lq->smoothed_lq.valueNlq/10) {
        relevant = true;
      }
    }
    else if (lq->smoothed_lq.valueNlq > lq->lq.valueNlq) {
      if (lq->smoothed_lq.valueNlq - lq->lq.valueNlq > lq->smoothed_lq.valueNlq/10) {
        relevant = true;
      }
    }

    if (relevant) {
      memcpy(&lq->smoothed_lq, &lq->lq, sizeof(struct default_lq_clm2));
      //Edited July 31, 2017
      link->linkcost = default_lq_calc_cost_clm2(&lq->smoothed_lq);
      //link->linkcost = 1;
      triggered = true;
    }
  } OLSR_FOR_ALL_LINK_ENTRIES_END(link)

  if (!triggered) {
    return;
  }

  OLSR_FOR_ALL_LINK_ENTRIES(link) {
    lq = (struct default_lq_clm2_hello *)link->linkquality;

    if (lq->smoothed_lq.valueLq == 255 && lq->smoothed_lq.valueNlq == 255) {
      continue;
    }

    if (lq->smoothed_lq.valueLq == lq->lq.valueLq && lq->smoothed_lq.valueNlq == lq->lq.valueNlq) {
      continue;
    }

    memcpy(&lq->smoothed_lq, &lq->lq, sizeof(struct default_lq_clm2));
    link->linkcost = default_lq_calc_cost_clm2(&lq->smoothed_lq);
  } OLSR_FOR_ALL_LINK_ENTRIES_END(link)

  olsr_relevant_linkcost_change();
}

static void
default_lq_parser_clm2(struct olsr *olsr, struct interface_olsr *in_if, union olsr_ip_addr *from_addr)
{
  const union olsr_ip_addr *main_addr;
  struct link_entry *lnk;
  struct default_lq_clm2_hello *lq;
  uint32_t seq_diff;

  /* Find main address */
  main_addr = mid_lookup_main_addr(from_addr);

  /* Loopup link entry */
  lnk = lookup_link_entry(from_addr, main_addr, in_if);
  if (lnk == NULL) {
    return;
  }

  lq = (struct default_lq_clm2_hello *)lnk->linkquality;

  /* ignore double package */
  if (lq->last_seq_nr == olsr->olsr_seqno) {
    struct ipaddr_str buf;
    olsr_syslog(OLSR_LOG_INFO, "detected duplicate packet with seqnr 0x%x from %s on %s (%d Bytes)",
		olsr->olsr_seqno,olsr_ip_to_string(&buf, from_addr),in_if->int_name,ntohs(olsr->olsr_packlen));
    return;
  }

  if (lq->last_seq_nr > olsr->olsr_seqno) {
    seq_diff = (uint32_t) olsr->olsr_seqno + 65536 - lq->last_seq_nr;
  } else {
    seq_diff = olsr->olsr_seqno - lq->last_seq_nr;
  }

  /* Jump in sequence numbers ? */
  if (seq_diff > 256) {
    seq_diff = 1;
  }

  lq->received[lq->activePtr]++;
  lq->total[lq->activePtr] += seq_diff;

  lq->last_seq_nr = olsr->olsr_seqno;
  lq->missed_hellos = 0;
}

static void
default_lq_clm2_timer(void __attribute__ ((unused)) * context)
{
  struct link_entry *link;

  uint8_t clm2_value = 0;
  get_clm2_stats();

  OLSR_FOR_ALL_LINK_ENTRIES(link) {
    struct default_lq_clm2_hello *tlq = (struct default_lq_clm2_hello *)link->linkquality;
    
  /*
	Modified: Jerome
	UNDER THE BAYANIHANETS PROJECT 3
	DEVELOPMENT OF ROUTING METRICS FOR HIGH THROUGHPUT
	AND CONGESTION FREE ROUTING OVER WIRELESS COMMUNITY MESH NETWORKS

	DATE: September 7, 2017
	PURPOSE: Store the computed statistics from get_clm2_stats() to the valueLq
  */
	if(clm2_cca_value > clm2_nav_value){
	      clm2_value = clm2_cca_value;
	} else {
	      clm2_value = clm2_nav_value;
    	}	

	true_lq_value = (uint8_t) ((double) (true_lq_value + clm2_value) / (double) 2);
	
	//October 3, 2017
	if(true_lq_value == 0){
		true_lq_value = 1;
	}

	tlq->lq.valueLq = true_lq_value;
  /*
	Modification ends here
  */

    // shift buffer
    tlq->activePtr = (tlq->activePtr + 1) % LQ_CLM2_WINDOW;
    tlq->total[tlq->activePtr] = 0;
    tlq->received[tlq->activePtr] = 0;
  } OLSR_FOR_ALL_LINK_ENTRIES_END(link);

  default_lq_clm2_handle_lqchange();
}

static void
default_lq_initialize_clm2(void)
{
  olsr_packetparser_add_function(&default_lq_parser_clm2);
  olsr_start_timer(1000, 0, OLSR_TIMER_PERIODIC, &default_lq_clm2_timer, NULL, 0);
}

static olsr_linkcost
default_lq_calc_cost_clm2(const void *ptr)
{
  const struct default_lq_clm2 *lq = ptr;
  olsr_linkcost cost;

  //if (lq->valueLq < (unsigned int)(255 * MINIMAL_USEFUL_LQ) || lq->valueNlq < (unsigned int)(255 * MINIMAL_USEFUL_LQ)) {
    //return LINK_COST_BROKEN;
  //}

  //cost = fpmidiv(itofpm(255 * 255), (int) lq->valueLq * (int) lq->valueNlq);
  cost = (int)((lq->valueLq + lq->valueNlq)/2);

  //printf("Cost: %" SCNu32 "; LQ: %" SCNu8 "; NLQ: %" SCNu8 "\n", cost, lq->valueLq, lq->valueNlq);

  if (cost > LINK_COST_BROKEN)
    return LINK_COST_BROKEN;
  if (cost == 0)
    return 1;
  return cost;
}

static int
default_lq_serialize_hello_lq_pair_clm2(unsigned char *buff, void *ptr)
{
  struct default_lq_clm2 *lq = ptr;
  
  //uint8_t clm2_value;

  /*
	Modified: Arsenius
	UNDER THE BAYANIHANETS PROJECT 3
	DEVELOPMENT OF ROUTING METRICS FOR HIGH THROUGHPUT
	AND CONGESTION FREE ROUTING OVER WIRELESS COMMUNITY MESH NETWORKS

	DATE: August 15, 2017
	PURPOSE: call get_clm2_stats to obtain the clm2_value and clm2_nav_value that will be used both by
	and the serialize_lq_tc functions.
  */
  //get_clm2_stats();
  /*
	Modification ends here.
  */

  /*
	Modified: Arsenius
	UNDER THE BAYANIHANETS PROJECT 3
	DEVELOPMENT OF ROUTING METRICS FOR HIGH THROUGHPUT
	AND CONGESTION FREE ROUTING OVER WIRELESS COMMUNITY MESH NETWORKS

	DATE: August 23, 2017
	PURPOSE: 
	(1) obtain the real value of cca and nav by adding their previous 
	    and present values and divide the results by two
	(2) assign the highest value to true_lq_value
	(3) assign true_lq to buff[0]
  */

  //if the clm2_reader is called for the first time
  //if(is_first == 1)
  //{
    //if(clm2_value > clm2_nav_value){
      //true_lq_value = clm2_value;
    //} else {
      //true_lq_value = clm2_nav_value;
    //}
  //} else {
    //obtain the real value of cca and nav by adding their previous 
    //and present values and divide the results by two
    //true_cca_value = (uint8_t) ((double) (previous_clm2_value + clm2_value) / (double) 2);
    //true_nav_value = (uint8_t) ((double) (previous_clm2_nav_value + clm2_nav_value) / (double) 2);

    //if(true_cca_value > true_nav_value){
      //true_lq_value = true_cca_value;
    //} else {
      //true_lq_value = true_nav_value;
    //}
  //}

  //printf("CCA_PREV: %" SCNu8 "; CCA_PRES: %" SCNu8 "; NAV_PREV: %" SCNu8 "; NAV_PRES: %" SCNu8 "\n", 
  //previous_clm2_value, clm2_value, previous_clm2_nav_value, clm2_nav_value);

  //printf("TRUE_CCA: %" SCNu8 "; TRUE_NAV: %" SCNu8 "\n", true_cca_value, true_nav_value);

  /*
	Modification ends here.
  */
  
  /*
	Modified: Jerome
	UNDER THE BAYANIHANETS PROJECT 3
	DEVELOPMENT OF ROUTING METRICS FOR HIGH THROUGHPUT
	AND CONGESTION FREE ROUTING OVER WIRELESS COMMUNITY MESH NETWORKS

	DATE: August 31, 2017
	PURPOSE: 
	(1) obtain the clm2_value (bigger between CCA and NAV values
	(2) calculate the true_lq_value (average of previous true_lq_value and current clm2_value)
  */
	//if(clm2_cca_value > clm2_nav_value){
	      //clm2_value = clm2_cca_value;
	//} else {
	      //clm2_value = clm2_nav_value;
    	//}
	//printf("PREV_TRUE_LQ: %" SCNu8 "; CLM2: %" SCNu8 "\n", true_lq_value, clm2_value);	
	//true_lq_value = (uint8_t) ((double) (true_lq_value + clm2_value) / (double) 2);
	//printf("CURR_TRUE_LQ: %" SCNu8 "\n", true_lq_value);
  /*
	Modification ends here
  */
  
  //buff[0] = (unsigned char) (255 - true_lq_value);
  buff[0] = (unsigned char)lq->valueLq;
  buff[1] = (unsigned char)lq->valueNlq;
  buff[2] = (unsigned char)(0);
  buff[3] = (unsigned char)(0);

  //printf("Serialize_LQ LQ: %" SCNu8 " : NLQ: %" SCNu8 "\n", true_lq_value, lq->valueNlq);

  return 4;
}

static void
default_lq_deserialize_hello_lq_pair_clm2(const uint8_t ** curr, void *ptr)
{
  struct default_lq_clm2 *lq = ptr;


  pkt_get_u8(curr, &lq->valueLq);
  pkt_get_u8(curr, &lq->valueNlq);

  
 /*
	Modified: Jerome
	UNDER THE BAYANIHANETS PROJECT 3
	DEVELOPMENT OF ROUTING METRICS FOR HIGH THROUGHPUT
	AND CONGESTION FREE ROUTING OVER WIRELESS COMMUNITY MESH NETWORKS

	DATE: August 10, 2017
	PURPOSE: header file to allow to print uint8_t
 */
 	//pkt_get_u8(curr, &lq->dummy);
 	//printf("Deserialize_LQ LQ: %" SCNu8 ": NLQ: %" SCNu8 "\n", lq->valueLq, lq->valueNlq);
 /*
	Modification ends here.
 */

  pkt_ignore_u16(curr);
}

static int
default_lq_serialize_tc_lq_pair_clm2(unsigned char *buff, void *ptr)
{
  struct default_lq_clm2 *lq = ptr;
   
  /*
	Modified: Anon.
	UNDER THE BAYANIHANETS PROJECT 3
	DEVELOPMENT OF ROUTING METRICS FOR HIGH THROUGHPUT
	AND CONGESTION FREE ROUTING OVER WIRELESS COMMUNITY MESH NETWORKS

	DATE: August 15, 2017
	PURPOSE: Assign the clm2_value that was previously supplied using serialize_lq_hello
	to get the same value as the hello message
  */
  //clm2_value = get_clm2_stats();
//  buff[0] = (unsigned char)true_lq_value;
  /*
	Modification ends here.
  */
  buff[0] = (unsigned char)lq->valueLq;
  buff[1] = (unsigned char)lq->valueNlq;
  buff[2] = (unsigned char)(0);
  buff[3] = (unsigned char)(0);

  //printf("Serialize_TC LQ: %" SCNu8 " : NLQ: %" SCNu8 "\n", clm2_cca_value, lq->valueNlq);

  return 4;
}

static void
default_lq_deserialize_tc_lq_pair_clm2(const uint8_t ** curr, void *ptr)
{
  struct default_lq_clm2 *lq = ptr;
  pkt_get_u8(curr, &lq->valueLq);
  pkt_get_u8(curr, &lq->valueNlq);
  /*
	Modified: Anon
	UNDER THE BAYANIHANETS PROJECT 3
	DEVELOPMENT OF ROUTING METRICS FOR HIGH THROUGHPUT
	AND CONGESTION FREE ROUTING OVER WIRELESS COMMUNITY MESH NETWORKS

	DATE: August 10, 2017
	PURPOSE: header file to allow to print uint8_t
 */
 	//pkt_get_u8(curr, &lq->dummy);
	//printf("Deserialize LQ: %" SCNu8 ": NLQ: %" SCNu8 "\n", lq->valueLq, lq->valueNlq);
 /*
	Modification ends here.
 */  

  pkt_ignore_u16(curr);
}

static void
default_lq_packet_loss_worker_clm2(struct link_entry *link,
    void __attribute__ ((unused)) *ptr, bool lost)
{
  struct default_lq_clm2_hello *tlq = (struct default_lq_clm2_hello *)link->linkquality;

  if (lost) {
    tlq->missed_hellos++;
  }
  return;
}

static void
default_lq_memorize_foreign_hello_clm2(void *ptrLocal, void *ptrForeign)
{
  struct default_lq_clm2_hello *local = ptrLocal;
  struct default_lq_clm2 *foreign = ptrForeign;

  if (foreign) {
    local->lq.valueNlq = foreign->valueLq;
  } else {
    local->lq.valueNlq = 0;
  }
}

static void
default_lq_copy_link2neigh_clm2(void *t, void *s)
{
  struct default_lq_clm2 *target = t;
  struct default_lq_clm2_hello *source = s;
  *target = source->smoothed_lq;
}

static void
default_lq_copy_link2tc_clm2(void *t, void *s)
{
  struct default_lq_clm2 *target = t;
  struct default_lq_clm2_hello *source = s;
  *target = source->smoothed_lq;
}

static void
default_lq_clear_clm2(void *target)
{
  memset(target, 0, sizeof(struct default_lq_clm2));
}

static void
default_lq_clear_clm2_hello(void *target)
{
  struct default_lq_clm2_hello *local = target;
  int i;

  default_lq_clear_clm2(&local->lq);
  default_lq_clear_clm2(&local->smoothed_lq);
  local->windowSize = LQ_CLM2_QUICKSTART_INIT;
  for (i = 0; i < LQ_CLM2_WINDOW; i++) {
    local->total[i] = 3;
  }
}

static const char *
default_lq_print_clm2(void *ptr, char separator, struct lqtextbuffer *buffer)
{
  struct default_lq_clm2 *lq = ptr;

  //snprintf(buffer->buf, sizeof(buffer->buf), "%s%c%s", fpmtoa(fpmidiv(itofpm((int)lq->valueLq), 255)), separator,
  //         fpmtoa(fpmidiv(itofpm((int)lq->valueNlq), 255)));
/*
	Modified: Arsenius
	UNDER THE BAYANIHANETS PROJECT 3
	DEVELOPMENT OF ROUTING METRICS FOR HIGH THROUGHPUT
	AND CONGESTION FREE ROUTING OVER WIRELESS COMMUNITY MESH NETWORKS

	DATE: September 7, 2017
	PURPOSE: modifies the text outputted to dashboard to account for 
	clm2 statistics rather than ETX statistics.
*/
  snprintf(buffer->buf, sizeof(buffer->buf), "%lu%c%lu", (long)lq->valueLq, separator,
           (long)lq->valueNlq);
  return buffer->buf;
}
/*
	Modification ends here.
*/
static const char *
default_lq_print_cost_clm2(olsr_linkcost cost, struct lqtextbuffer *buffer)
{
  /*
	Modified: Arsenius
	UNDER THE BAYANIHANETS PROJECT 3
	DEVELOPMENT OF ROUTING METRICS FOR HIGH THROUGHPUT
	AND CONGESTION FREE ROUTING OVER WIRELESS COMMUNITY MESH NETWORKS

	DATE: September 7, 2017
	PURPOSE: modifies the text outputted to dashboard to account for 
	clm2 statistics rather than ETX statistics.
*/
  //snprintf(buffer->buf, sizeof(buffer->buf), "%s", fpmtoa((fpm) cost));
  snprintf(buffer->buf, sizeof(buffer->buf), "%lu", (long) cost);
  return buffer->buf;
/*
	Modification ends here.
*/
}

/*
	Modified: Arsenius
	UNDER THE BAYANIHANETS PROJECT 3
	DEVELOPMENT OF ROUTING METRICS FOR HIGH THROUGHPUT
	AND CONGESTION FREE ROUTING OVER WIRELESS COMMUNITY MESH NETWORKS

	DATE: August 14, 2017
	PURPOSE: implement dummy function to return a static value
	This is useful to augment clm2_presenter later when this is successfully implemented
*/
static void
get_clm2_stats(void) {
	sock_fd2 = socket(PF_NETLINK, SOCK_RAW, NETLINK_USER);
    	if (sock_fd2 < 0)
        	return;
	
    	memset(&src_addr2, 0, sizeof(src_addr2));
    	src_addr2.nl_family = AF_NETLINK;
    	src_addr2.nl_pid = getpid(); /* self pid */
	
    	bind(sock_fd2, (struct sockaddr *)&src_addr2, sizeof(src_addr2));
	
    	memset(&dest_addr2, 0, sizeof(dest_addr2));
    	memset(&dest_addr2, 0, sizeof(dest_addr2));
    	dest_addr2.nl_family = AF_NETLINK;
    	dest_addr2.nl_pid = 0; /* For Linux Kernel */
    	dest_addr2.nl_groups = 0; /* unicast */
	
    	nlh2 = (struct nlmsghdr *) malloc(NLMSG_SPACE(MAX_PAYLOAD));
    	memset(nlh2, 0, NLMSG_SPACE(MAX_PAYLOAD));
    	nlh2->nlmsg_len = NLMSG_SPACE(MAX_PAYLOAD);
    	nlh2->nlmsg_pid = getpid();
    	nlh2->nlmsg_flags = 0;
	
    	strcpy(NLMSG_DATA(nlh2), "Hello");

    	iov2.iov_base = (void *)nlh2;
    	iov2.iov_len = nlh2->nlmsg_len;
    	msg2.msg_name = (void *)&dest_addr2;
    	msg2.msg_namelen = sizeof(dest_addr2);
    	msg2.msg_iov = &iov2;
    	msg2.msg_iovlen = 1;
	
	//Do the call to clm2
	sendmsg(sock_fd2, &msg2, 0);
	
	/* Read message from kernel */
    	recvmsg(sock_fd2, &msg2, 0);

	sscanf(NLMSG_DATA(nlh2), "%llu %llu %llu %llu", &clm2_measurement_time, 
	&clm2_busy_time, &clm2_nav_read_counter, &clm2_nav_busy_counter);

	//printf("CLM2_BUSYNESS: %" SCNu8 "\n", (uint8_t) (((double) clm2_busy_time / (double) clm2_measurement_time) * 255));
		
	//clm2_cca_value = ((uint8_t) (((double) clm2_busy_time / (double) clm2_measurement_time) * 255));
	//clm2_nav_value = ((uint8_t) (((double) clm2_nav_busy_counter / (double) clm2_nav_read_counter) * 255));

	clm2_cca_value = ((uint8_t) (((double) clm2_busy_time / (double) clm2_measurement_time) * 25));
	clm2_nav_value = ((uint8_t) (((double) clm2_nav_busy_counter / (double) clm2_nav_read_counter) * 25));


    	close(sock_fd2);

	//Free allocated resources
	free(nlh2);
	
	//print the computed NAV and CCA value
	//printf("CLM2: %" SCNu8 "; CLM2_NAV: %" SCNu8 "\n", clm2_cca_value, clm2_nav_value);	
}
/*
	Modification ends here
*/

/*
 * Local Variables:
 * c-basic-offset: 2
 * indent-tabs-mode: nil
 * End:
 */



