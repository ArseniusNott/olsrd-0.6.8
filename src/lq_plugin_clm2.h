
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

#ifndef LQ_CLM2_
#define LQ_CLM2_

#include "olsr_types.h"
#include "lq_plugin.h"

#define LQ_PLUGIN_LC_MULTIPLIER 1024
#define LQ_PLUGIN_RELEVANT_COSTCHANGE_CLM2 16

#define LQ_ALGORITHM_CLM2_NAME "clm2"

#define LQ_CLM2_WINDOW 32
#define LQ_CLM2_QUICKSTART_INIT 4

struct default_lq_clm2 {
  uint8_t valueLq;
  uint8_t valueNlq;
};

struct default_lq_clm2_hello {
  struct default_lq_clm2 smoothed_lq;
  struct default_lq_clm2 lq;
  uint8_t windowSize, activePtr;
  uint16_t last_seq_nr;
  uint16_t missed_hellos;
  uint16_t received[LQ_CLM2_WINDOW], total[LQ_CLM2_WINDOW];
};
//edited July 1, 2017
//extern struct lq_handler lq_etx_ff_handler;
struct lq_handler lq_clm2_handler;

#endif /* LQ_ETX_FF_ */

/*
 * Local Variables:
 * c-basic-offset: 2
 * indent-tabs-mode: nil
 * End:
 */


