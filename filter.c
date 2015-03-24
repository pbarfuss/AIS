/*
 *    filter.c  --  FIR filter
 *
 *    Copyright (C) 2001, 2002, 2003
 *      Tomi Manninen (oh2bns@sral.fi)
 *
 *    This file is part of gMFSK.
 *
 *    gMFSK is free software; you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation; either version 2 of the License, or
 *    (at your option) any later version.
 *
 *    gMFSK is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with gMFSK; if not, write to the Free Software
 *    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "filter.h"
//#include "fdsp.h"

static float rcos_filter_coeffs[]={
   2.5959e-55, 2.9479e-49, 1.4741e-43, 3.2462e-38, 3.1480e-33,
   1.3443e-28, 2.5280e-24, 2.0934e-20, 7.6339e-17, 1.2259e-13,
   8.6690e-11, 2.6996e-08, 3.7020e-06, 2.2355e-04, 5.9448e-03,
   6.9616e-02, 3.5899e-01, 8.1522e-01, 8.1522e-01, 3.5899e-01,
   6.9616e-02, 5.9448e-03, 2.2355e-04, 3.7020e-06, 2.6996e-08,
   8.6690e-11, 1.2259e-13, 7.6339e-17, 2.0934e-20, 2.5280e-24,
   1.3443e-28, 3.1480e-33, 3.2462e-38, 1.4741e-43, 2.9479e-49,
   2.5959e-55
};
#define COEFFS_L 36

/* ---------------------------------------------------------------------- */

void filter_init(struct filter *f)
{
	memset(f, 0, sizeof(struct filter));
	f->pointer = COEFFS_L;
}

void filter_free(struct filter *f)
{
}

/* ---------------------------------------------------------------------- */

void filter_run(struct filter *f, float in, float *out)
{
	float *ptr = f->buffer + f->pointer++;
	*ptr = in;
	*out = scalarproduct_float_sse(ptr - COEFFS_L, rcos_filter_coeffs, COEFFS_L);
	//*out = filter_mac(ptr - COEFFS_L, rcos_filter_coeffs, 53);
	if (f->pointer == BufferLen) {
		memcpy(f->buffer,
		       f->buffer + BufferLen - COEFFS_L,
		       COEFFS_L * sizeof(float));
		f->pointer = COEFFS_L;
	}
}

short filter_run_buf(struct filter *f, short *in, float *out, int step, int len)
{
	int id = 0;
	int od = 0;
	short maxval = 0;
	int pointer = f->pointer;
	float *buffer = f->buffer;

	while (od < len) {
	        buffer[pointer] = in[id];

		// look for peak volume
		if (in[id] > maxval)
			maxval = in[id];

		out[od] = scalarproduct_float_sse(&buffer[pointer - COEFFS_L], rcos_filter_coeffs, COEFFS_L);
		pointer++;

		/* the buffer is much smaller than the incoming chunks */
		if (pointer == BufferLen) {
			memcpy(buffer,
			       buffer + BufferLen - COEFFS_L,
			       COEFFS_L * sizeof(float));
			pointer = COEFFS_L;
		}

		id += step;
		od++;
	}

	f->pointer = pointer;
	return maxval;
}

/* ---------------------------------------------------------------------- */
