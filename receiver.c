
/*
 *	receiver.c
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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "receiver.h"
#define	INC	16
#define FILTERED_LEN 4096
unsigned int sound_levellog = 0;

#if 0
static float rcos_filter_coeffs[]={
   2.5959e-55, 2.9479e-49, 1.4741e-43, 3.2462e-38,
   3.1480e-33, 1.3443e-28, 2.5280e-24, 2.0934e-20,
   7.6339e-17, 1.2259e-13, 8.6690e-11, 2.6996e-08,
   3.7020e-06, 2.2355e-04, 5.9448e-03, 6.9616e-02,
   3.5899e-01, 8.1522e-01, 8.1522e-01, 3.5899e-01,
   6.9616e-02, 5.9448e-03, 2.2355e-04, 3.7020e-06,
   2.6996e-08, 8.6690e-11, 1.2259e-13, 7.6339e-17,
   2.0934e-20, 2.5280e-24, 1.3443e-28, 3.1480e-33,
   3.2462e-38, 1.4741e-43, 2.9479e-49, 2.5959e-55
};
#define COEFFS_L 36
#endif

static float rcos_filter_coeffs[84] = {
    -0.0195706589, -0.0019463516, +0.0174879305, +0.0376500445,
    +0.0573162726, +0.0751855693, +0.0899533523, +0.1003910752,
    +0.1054272579, +0.1042253298, +0.0962535644, +0.0813425924,
    +0.0597264537, +0.0320638792, -0.0005625513, -0.0366705982,
    -0.0744258758, -0.1117113668, -0.1462163400, -0.1755408960,
    -0.1973113466, -0.2093008549, -0.2095492535, -0.1964757729,
    -0.1689785575, -0.1265153337, -0.0691603972, +0.0023658260,
    +0.0866970749, +0.1818523559, +0.2852972395, +0.3940331739,
    +0.5047111565, +0.6137647860, +0.7175566441, +0.8125311731,
    +0.8953667896, +0.9631199126, +1.0133539072, +1.0442466315,
    +1.0546712948, +1.0442466315, +1.0133539072, +0.9631199126,
    +0.8953667896, +0.8125311731, +0.7175566441, +0.6137647860,
    +0.5047111565, +0.3940331739, +0.2852972395, +0.1818523559,
    +0.0866970749, +0.0023658260, -0.0691603972, -0.1265153337,
    -0.1689785575, -0.1964757729, -0.2095492535, -0.2093008549,
    -0.1973113466, -0.1755408960, -0.1462163400, -0.1117113668,
    -0.0744258758, -0.0366705982, -0.0005625513, +0.0320638792,
    +0.0597264537, +0.0813425924, +0.0962535644, +0.1042253298,
    +0.1054272579, +0.1003910752, +0.0899533523, +0.0751855693,
    +0.0573162726, +0.0376500445, +0.0174879305, -0.0019463516,
    -0.0195706589, 0.0, 0.0, 0.0
};
#define COEFFS_L 81
#define INVGAIN 0.7f

/* ---------------------------------------------------------------------- */

float filter_run_buf(ais_receiver_t *f, float *in, float *out, unsigned int len)
{
    float maxval = 0.0f;
    unsigned int id = 0, filtidx = f->rrc_filter_bufidx;

	while (id < len) {
	    f->rrc_filter_buffer[f->rrc_filter_bufidx] = in[id];

		// look for peak volume
		if (in[id] > maxval)
			maxval = in[id];

		out[id++] = INVGAIN*scalarproduct_float_sse(&f->rrc_filter_buffer[filtidx - COEFFS_L], rcos_filter_coeffs, 84);

		/* the buffer is much smaller than the incoming chunks */
		if (filtidx == RRC_BUFLEN-1) {
			memcpy(f->rrc_filter_buffer,
			       f->rrc_filter_buffer + RRC_BUFLEN - COEFFS_L,
			       COEFFS_L * sizeof(float));
            filtidx = COEFFS_L - 1;
		}
        filtidx++;
	}

    f->rrc_filter_bufidx = filtidx;
	return maxval;
}

void init_ais_receiver(ais_receiver_t *rx, int nmea_out_fd)
{
	memset(rx, 0, sizeof(ais_receiver_t));

	protodec_initialize(&rx->decoder, nmea_out_fd);
	rx->rrc_filter_bufidx = COEFFS_L;
	rx->lastbit = 0;
	rx->pll = 0;
	rx->pllinc = 0x10000 / 5;
	rx->prev = 0;
	rx->last_levellog = 0;
}

void ais_receiver_run(ais_receiver_t *rx, float *buf, unsigned int len)
{
	float out, maxval = 0.0f;
    unsigned int i;
	int curr, bit;
	char b;
	int level_distance;
	float level;
	float filtered[FILTERED_LEN];

	/* len is number of samples available in buffer for each
	 * channels - something like 1024, regardless of number of channels */

	if (len > FILTERED_LEN) {
		printf("receiver_run: input length %u would overflow buffer!\n", len);
        return;
    }

	maxval = filter_run_buf(rx, buf, filtered, len);
	for (i = 0; i < len; i++) {
		out = filtered[i];
		curr = (out > 0);

		if ((curr ^ rx->prev) == 1) {
			if (rx->pll < (0x10000 / 2)) {
				rx->pll += rx->pllinc / INC;
			} else {
				rx->pll -= rx->pllinc / INC;
			}
		}
		rx->prev = curr;
		rx->pll += rx->pllinc;

		if (rx->pll > 0xffff) {
			/* slice */
			bit = (out > 0);
			/* nrzi decode */
			b = !(bit ^ rx->lastbit);
			/* feed to the decoder */
			protodec_decode(&b, 1, &rx->decoder);
			rx->lastbit = bit;
			rx->pll &= 0xffff;
		}
	}

	/* calculate level, and log it */
	level = maxval * (float)100;
	level_distance = time(NULL) - rx->last_levellog;

	if (level > 95.0f && (level_distance >= 30 || level_distance >= sound_levellog)) {
		printf("Level too high: %.0f %%", level);
		time(&rx->last_levellog);
	} else if (sound_levellog != 0 && level_distance >= sound_levellog) {
        printf("Level: %.0f %%", level);
		time(&rx->last_levellog);
	}
}

