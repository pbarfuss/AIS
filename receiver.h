
/*
 *	receiver.h
 *
 *	(c) Ruben Undheim 2008
 *	(c) Heikki Hannikainen 2008
 *
 *    This program is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */


#ifndef INC_RECEIVER_H
#define INC_RECEIVER_H

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "protodec.h"
#define RRC_BUFLEN 1024

/* ---------------------------------------------------------------------- */

typedef struct _ais_receiver_t {
    float rrc_filter_buffer[RRC_BUFLEN];
    unsigned int rrc_filter_bufidx;
	int lastbit;
	unsigned int pll;
	unsigned int pllinc;
	struct demod_state_t decoder;
	int prev;
	time_t last_levellog;
} ais_receiver_t;

extern void init_ais_receiver(ais_receiver_t *rx, int nmea_out_fd);
extern float filter_run_buf(ais_receiver_t *f, float *in, float *out, unsigned int len);
extern void ais_receiver_run(ais_receiver_t *rx, float *buf, unsigned int len);

#endif
