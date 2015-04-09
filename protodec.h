
/*
 *	protodec.h
 *
 *	(c) Ruben Undheim 2008
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


#ifndef INC_PROTODEC_H
#define INC_PROTODEC_H

#define ST_SKURR 1
#define ST_PREAMBLE 2
#define ST_STARTSIGN 3
#define ST_DATA 4
#define ST_STOPSIGN 5
#define MAX_AIS_PACKET_TYPE 24
#define DEMOD_BUFFER_LEN 1024

// change this to
//#define DBG(x) x
// if you want to see all debug text
#define DBG(x)
#define SERBUFFER_LEN 128

struct demod_state_t {
	int state;
	unsigned int offset;
	int nskurr, npreamble, nstartsign, ndata, nstopsign;

	int antallenner;
	unsigned char buffer[DEMOD_BUFFER_LEN];
	unsigned char rbuffer[DEMOD_BUFFER_LEN];
	unsigned int bufferpos;
	char last;
	int antallpreamble;
	int bitstuff;
	unsigned int receivedframes;
	unsigned int lostframes;
	unsigned int lostframes2;
	unsigned char seqnr;
	float best_range;

    int nmea_out_fd;
    char nmea_buffer[SERBUFFER_LEN];
};

void protodec_initialize(struct demod_state_t *d, int serial_out_fd);
void protodec_reset(struct demod_state_t *d);
void protodec_getdata(unsigned int buffer_len, struct demod_state_t *d);
void protodec_decode(unsigned char *in, unsigned int count, struct demod_state_t *d);

#endif
