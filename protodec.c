
/*
 *	protodec.c
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

#include <stdint.h>
#include <stdlib.h>
#include <string.h>		/* String function definitions */
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <unistd.h>
#include "protodec.h"
#define RAD_2_DEG 57.295779513f
#define DEG_2_RAD  0.0174532925f
static const char hex[]="0123456789ABCDEF";
static const float inv_pi  =  0.3183098733;  /* 0x3ea2f984 */

float mylat =  44.635225f;
float mylng = -63.593534f;

void protodec_initialize(struct demod_state_t *d, int serial_out_fd)
{
	memset(d, 0, sizeof(struct demod_state_t));
    d->nmea_out_fd = serial_out_fd;
	d->receivedframes = 0;
	d->lostframes = 0;
	d->lostframes2 = 0;
	d->seqnr = 0;
}

void protodec_reset(struct demod_state_t *d)
{
	d->state = ST_SKURR;
	d->nskurr = 0;
	d->ndata = 0;
	d->npreamble = 0;
	d->nstartsign = 0;
	d->nstopsign = 0;
	d->antallpreamble = 0;
	d->antallenner = 0;
	d->last = 0;
	d->bitstuff = 0;
	d->bufferpos = 0;
}

/*
 * Calculates CRC-checksum
 *
 * This is CRC16-CCITT, used all over the place in broadcast
 * (for instance, PACTOR and DMR, for just two examples).
 * TODO: replace with a table-based version that does byte-at-a-time processing.
 *
 */

unsigned short protodec_sdlc_crc(const unsigned char *data, unsigned len)
{
	unsigned short c, crc = 0xffff;

	while (len--) {
		for (c = 0x100 + *data++; c > 1; c >>= 1)
			if ((crc ^ c) & 1)
				crc = (crc >> 1) ^ 0x8408;
			else
				crc >>= 1;
    }
	return ~crc;
}

/*
 * FIXME: remove the silly bitreversal into a separate buffer crap.
 */
int protodec_calculate_crc(unsigned int length_bits, struct demod_state_t *d)
{
	unsigned char buf[128+3], tmp;
	unsigned int i, j, length_bytes = (length_bits >> 3);
    unsigned short crc, stored_crc = 0;

	/* what is this?
     * bit-packing, you sillies -- bofh
     */
	for (j = 0; j < length_bytes; j++) {
		tmp = 0;
		for (i = 0; i < 8; i++)
			tmp |= ((d->buffer[8*j+i]) << i);
		buf[j] = tmp;
	}

	/* ok, here's the actual CRC calculation */
	for (i = 0; i < 16; i++)
		stored_crc |= ((d->buffer[8*length_bytes+i]) << i);
	crc = protodec_sdlc_crc(buf, length_bytes);
	printf("CRC: stored: 0x%04x, computed: 0x%04x\n", stored_crc, crc);

	for (j = 0; j < length_bytes; j++) {
		for (i = 0; i < 8; i++) {
			//d->rbuffer[8*j+7-i] = d->buffer[8*j+i];
			d->rbuffer[8*j+i] = (buf[j] >> (7 - i)) & 1;
		}
	}
    for (j = 0; j < DEMOD_BUFFER_LEN; j++) {
        d->buffer[j] = 0;
    }

	//return (crc == 0x0f47);
	return (crc == stored_crc);
}

/*
 *	Mark trailing spaces as NULL bytes in a string
 */

static void remove_trailing_spaces(char *s, int len)
{
	int i;

	s[len] = 0;
	for (i = len-1; i >= 0; i--) {
		if (s[i] == ' ' || s[i] == 0)
			s[i] = 0;
		else
			i = -1;
	}
}

/*
 *	Decode 6-bit ASCII to normal 8-bit ASCII
 */

void protodec_decode_sixbit_ascii(char sixbit, char *name, unsigned int pos)
{
	if (sixbit >= 1 && sixbit <= 31) {
		name[pos] = sixbit + 64;
		return;
	}

	if (sixbit >= 32 && sixbit <= 63) {
		name[pos] = sixbit;
		return;
	}

	name[pos] = ' ';
}

unsigned int protodec_henten(unsigned int from, unsigned int size, unsigned char *frame)
{
	unsigned int i, tmp = 0;
	for (i = 0; i < size; i++)
		tmp |= (frame[from + i]) << (size - 1 - i);
	return tmp;
}

/*
 *	binary message types
 */

const char *appid_ifm(unsigned int i)
{
	switch (i) {
	case 0:
		return "text-telegram";
	case 1:
		return "application-ack";
	case 2:
		return "iai-fi-capab-interrogation";
	case 3:
		return "iai-capabi-interrogation";
	case 4:
		return "capability-reply";
	case 11:
		return "tide-weather";
	case 16:
		return "vts-targets";
	case 17:
		return "ship-waypoints";
	case 18:
		return "advice-of-waypoints";
	case 19:
		return "extended-ship-data";
	case 20:
		return "berthing-data";
	case 21:
		return "weather-obs-report";
	case 22:
		return "area-notice-bc";
	case 23:
		return "area-notice-addr";
	case 24:
		return "extended-ship-static";
	case 25:
		return "dangerous-cargo-info";
	case 26:
		return "environmental";
	case 27:
		return "route-info-bc";
	case 28:
		return "route-info-addr";
	case 29:
		return "text-description-bc";
	case 30:
		return "text-description-addr";
	case 40:
		return "persons-on-board";
	default:
		return "unknown";
	}

	return "unknown";
}

/*
 *	binary message decoding
 */

void protodec_msg_40(unsigned char *buffer, unsigned int bufferlen, unsigned int msg_start)
{
	int people_on_board = protodec_henten(msg_start, 13, buffer);
	printf(" persons-on-board %d", people_on_board);
}

void protodec_msg_11(unsigned char *buffer, unsigned int bufferlen, unsigned int msg_start, time_t received_t, uint32_t mmsi)
{
	int latitude = protodec_henten(msg_start, 24, buffer);
	int longitude = protodec_henten(msg_start+24, 25, buffer);
	int wind_speed = protodec_henten(msg_start+40, 7, buffer);
	int wind_gust = protodec_henten(msg_start+47, 7, buffer);
	int wind_dir = protodec_henten(msg_start+54, 9, buffer);
	int wind_gust_dir = protodec_henten(msg_start+63, 9, buffer);
	int air_temp = protodec_henten(msg_start+72, 11, buffer);
	int rel_humid = protodec_henten(msg_start+83, 7, buffer);
	int dew_point = protodec_henten(msg_start+90, 10, buffer);
	int air_press = protodec_henten(msg_start+100, 9, buffer) + 800;
	int air_press_tend = protodec_henten(msg_start+109, 2, buffer);
	int horiz_visib_nm = protodec_henten(msg_start+111, 8, buffer);

	printf(" lat %.6f lon %.6f wind_speed %dkt wind_gust %dkt wind_dir %d wind_gust_dir %d"
           "air_temp %.1fC rel_humid %d%% dew_point %.1fC pressure %d pressure_tend %d visib %dNM\n",
		   (float)latitude / 60000.0f, (float)longitude / 60000.0f, wind_speed, wind_gust, wind_dir, wind_gust_dir,
		   (float)air_temp * 0.1f - 60.0f, rel_humid, (float)dew_point * 0.1f - 20.0f, air_press, air_press_tend, horiz_visib_nm);

	/* int water_level = protodec_henten(msg_start += 8, 9, buffer);
	 * int water_trend = protodec_henten(msg_start += 9, 2, buffer);
	 * int surface_current_speed = protodec_henten(msg_start += 2, 8, buffer);
	 * int surface_current_dir = protodec_henten(msg_start += 8, 9, buffer);
	 * int d1_current_speed = protodec_henten(msg_start += 9, 8, buffer);
	 * int d1_current_dir = protodec_henten(msg_start += 8, 9, buffer);
	 * int d1_current_depth = protodec_henten(msg_start += 9, 5, buffer);
	 * int d2_current_speed = protodec_henten(msg_start += 5, 8, buffer);
	 * int d2_current_dir = protodec_henten(msg_start += 8, 9, buffer);
	 * int d2_current_depth = protodec_henten(msg_start += 9, 5, buffer);
	 * int wave_height_significant = protodec_henten(msg_start += 5, 8, buffer);
	 * int wave_period = protodec_henten(msg_start += 8, 6, buffer);
	 * int wave_dir = protodec_henten(msg_start += 6, 9, buffer);
	 * int swell_height = protodec_henten(msg_start += 9, 8, buffer);
	 * int swell_period = protodec_henten(msg_start += 8, 6, buffer);
	 * int swell_dir = protodec_henten(msg_start += 6, 9, buffer);
	 * int sea_state = protodec_henten(msg_start += 9, 4, buffer);
	 * int water_temp = protodec_henten(msg_start += 4, 10, buffer);
     * printf("water_level %.1fm wave_height %.1fm water_temp %.1fC\n",
     *        (float)water_level * 0.1f - 10.0f, (float)wave_height_significant * 0.1f, (float)water_temp * 0.1f - 10.0f);
     */
}

void protodec_msg_bin(unsigned char *buffer, unsigned int bufferlen,
                      unsigned int appid_fi, int msg_start, time_t received_t,
                      uint32_t mmsi)
{
	switch (appid_fi) {
	case 11: // weather
		protodec_msg_11(buffer, bufferlen, msg_start, received_t, mmsi);
		break;
	case 40: // number of persons on board
		protodec_msg_40(buffer, bufferlen, msg_start);
		break;
	default:
		break;
	}
}

static const float
C1  = -5.0000000000e-01,
C2  =  4.1666667908e-02, /* 0x3d2aaaab */
C3  = -1.3888889225e-03, /* 0xbab60b61 */
C4  =  2.4801587642e-05, /* 0x37d00d01 */
C5  = -2.7557314297e-07, /* 0xb493f27c */
C6  =  2.0875723372e-09, /* 0x310f74f6 */
C7  = -1.1359647598e-11; /* 0xad47d74e */

// Differs from libc cosf on [0, pi/2] by at most 0.0000001229f
// Differs from libc cosf on [0, pi] by at most 0.0000035763f
static inline float k_cosf(float x)
{
    float z = x*x;
    return (1.0f+z*(C1+z*(C2+z*(C3+z*(C4+z*(C5+z*(C6+z*C7)))))));
}

float fast_cosf(float x) {
    float y = fabsf(x), z;
    uint32_t n = (uint32_t)(y*inv_pi);
    z = k_cosf(y - (float)M_PI*(float)n);
    return ((n&1) ? -z : z);
}

#if 0
float fast_asinf(float x) {
    float z;
    if (x > 0.5f) {
        float x2 = (1.0f - x);
        z = 0.5f*((float)M_PI - (float)M_SQRT2*sqrtf(x2)*(1.0f + 0.25f*x2*(0.16666666666f + 0.0375f*x2)));
    } else {
        float x2 = x*x;
        z = x*(1.0f + x2*(0.16666666666f + 0.075f*x2));
    }
    return z;
}
#endif

float fast_asinf(float x) {
    float z;
    float x2 = x*x;
    if (x > (float)M_SQRT1_2) {
        x2 = (1.0f - x2);
        z = 0.5f*(float)M_PI - sqrtf(x2)*(1.0f + x2*(0.16666666666f + 0.075f*x2));
    } else {
        z = x*(1.0f + x2*(0.16666666666f + 0.075f*x2));
    }
    return z;
}

float maidenhead_km_distance(float lat1, float lon1, float lat2, float lon2)
{
	float sindlat = (1.0f - fast_cosf(lat1 - lat2));
	float sindlon = (1.0f - fast_cosf(lon1 - lon2));
	float coslat1 = fast_cosf(lat1);
	float coslat2 = fast_cosf(lat2);
	float a = (float)M_SQRT1_2*sqrtf(sindlat + sindlon * coslat1 * coslat2);
	return (222.4f * fast_asinf(a));
}

static inline void update_range(struct demod_state_t *d, uint32_t mmsi, float lat, float lon)
{
    float distance;

    if (lat != lat) lat = 0.0f; // goddamn NaNs
    if (lon != lon) lon = 0.0f; // goddamn NaNs

	// ignore bad GPS fixes, sent commonly by some AIS stations
	if (fabsf(lat) > 89.0f || fabsf(lon) > 180.01f) {
        printf("invalid/erroneous latitude/longitude for MMSI %u: lat: %.5f, lon: %.5f\n", mmsi, lat, lon);
		return;
    }

	distance = RAD_2_DEG * maidenhead_km_distance(mylat, mylng, (lat * DEG_2_RAD), (lon * DEG_2_RAD));
	if (distance > d->best_range) {
        printf("updating range: old: %.5f, new: %.5f\n", d->best_range, distance);
		d->best_range = distance;
    }
}

/*
 *	decode position packets (types 1,2,3)
 */

void protodec_pos(struct demod_state_t *d, unsigned int bufferlen, time_t received_t, uint32_t mmsi)
{
	int longitude, latitude;
	float longit, latit;
	unsigned short course, sog, heading;
	char rateofturn, navstat;

	longitude = protodec_henten(61, 28, d->rbuffer);
	if (((longitude >> 27) & 1) == 1)
		longitude |= 0xF0000000;
	longit = ((float) longitude) * 0.0000005f;

	latitude = protodec_henten(61 + 28, 27, d->rbuffer);
	if (((latitude >> 26) & 1) == 1)
		latitude |= 0xf8000000;
	latit = ((float) latitude) * 0.0000005f;

	course = protodec_henten(61+ 28 + 27, 12, d->rbuffer);
	heading = protodec_henten(61 + 28 + 27 + 12, 9, d->rbuffer);
	sog = protodec_henten(50, 10, d->rbuffer);
	navstat = protodec_henten(38, 2, d->rbuffer);
	rateofturn = protodec_henten(38 + 2, 8, d->rbuffer);

	printf(" lat %.6f lon %.6f course %.0f speed %.1f rateofturn %d navstat %d heading %d",
		   latit / 3.0f, longit / 3.0f, (float) course * 0.1f, (float) sog * 0.1f, rateofturn, navstat, heading);
	update_range(d, mmsi, latit / 3.0f, longit / 3.0f);
}

void protodec_4(struct demod_state_t *d, unsigned int bufferlen, time_t received_t, uint32_t mmsi)
{
	unsigned int day, hour, minute, second, year, month;
	int longitude, latitude;
	float longit, latit;

	year = protodec_henten(40, 12, d->rbuffer);
	month = protodec_henten(52, 4, d->rbuffer);
	day = protodec_henten(56, 5, d->rbuffer);
	hour = protodec_henten(61, 5, d->rbuffer);
	minute = protodec_henten(66, 6, d->rbuffer);
	second = protodec_henten(72, 6, d->rbuffer);

	longitude = protodec_henten(79, 28, d->rbuffer);
	if (((longitude >> 27) & 1) == 1)
		longitude |= 0xF0000000;
	longit = ((float) longitude) * 0.0000005f;

	latitude = protodec_henten(107, 27, d->rbuffer);
	if (((latitude >> 26) & 1) == 1)
		latitude |= 0xf8000000;
	latit = ((float) latitude) * 0.0000005f;

	printf(" date %d-%d-%d time %02u:%02u:%02u lat %.6f lon %.6f",
		   year, month, day, hour, minute, second, latit / 3.0f, longit / 3.0f);
	update_range(d, mmsi, latit / 3.0f, longit / 3.0f);
}

void protodec_5(struct demod_state_t *d, unsigned int bufferlen, time_t received_t, uint32_t mmsi)
{
    unsigned int k, pos = 70, imo, shiptype;
	char callsign[7];
	char name[21];
	char destination[21];
	unsigned int A, B;
	unsigned char C, D;
	unsigned char draught;
	int letter;

	/* get IMO number */
	imo = protodec_henten(40, 30, d->rbuffer);
	printf("--- 5: mmsi %u imo %u\n", mmsi, imo);

	/* get callsign */
	for (k = 0; k < 6; k++) {
		letter = protodec_henten(pos, 6, d->rbuffer);
		protodec_decode_sixbit_ascii(letter, callsign, k);
		pos += 6;
	}
	callsign[6] = 0;
	remove_trailing_spaces(callsign, 6);

	/* get name */
	pos = 112;
	for (k = 0; k < 20; k++) {
		letter = protodec_henten(pos, 6, d->rbuffer);
		protodec_decode_sixbit_ascii(letter, name, k);
		pos += 6;
	}
	name[20] = 0;
	remove_trailing_spaces(name, 20);

	/* get destination */
	pos = 120 + 106 + 68 + 8;
	for (k = 0; k < 20; k++) {
		letter = protodec_henten(pos, 6, d->rbuffer);
		protodec_decode_sixbit_ascii(letter, destination, k);
		pos += 6;
	}
	destination[20] = 0;
	remove_trailing_spaces(destination, 20);

	/* type of ship and cargo */
	shiptype = protodec_henten(232, 8, d->rbuffer);

	/* dimensions and reference GPS position */
	A = protodec_henten(240, 9, d->rbuffer);
	B = protodec_henten(240 + 9, 9, d->rbuffer);
	C = protodec_henten(240 + 9 + 9, 6, d->rbuffer);
	D = protodec_henten(240 + 9 + 9 + 6, 6, d->rbuffer);
	draught = protodec_henten(294, 8, d->rbuffer);
	// printf("Length: %d\nWidth: %d\nDraught: %f\n",A+B,C+D,(float)draught/10);

	printf(" name \"%s\" destination \"%s\" type %d length %d width %d draught %.1f",
		   name, destination, shiptype, A + B, C + D, (float) draught * 0.1f);
}

/*
 *	6: addressed binary message
 */

void protodec_6(struct demod_state_t *d, unsigned int bufferlen, time_t received_t, uint32_t mmsi)
{
	int sequence = protodec_henten(38, 2, d->rbuffer);
	unsigned long dst_mmsi = protodec_henten(40, 30, d->rbuffer);
	int retransmitted = protodec_henten(70, 1, d->rbuffer);
	int appid = protodec_henten(72, 16, d->rbuffer);
	int appid_dac = protodec_henten(72, 10, d->rbuffer);
	int appid_fi = protodec_henten(82, 6, d->rbuffer);

	printf(" dst_mmsi %09ld seq %d retransmitted %d appid %d app_dac %d app_fi %d",
		   dst_mmsi, sequence, retransmitted, appid, appid_dac, appid_fi);

	if (appid_dac == 1) {
		printf("(%s)", appid_ifm(appid_fi));
		protodec_msg_bin(d->rbuffer, bufferlen, appid_fi, 88, received_t, mmsi);
	}
}

/*
 *	7: Binary acknowledge
 *	13: Safety related acknowledge
 */

void protodec_7_13(struct demod_state_t *d, unsigned int bufferlen, time_t received_t, uint32_t mmsi)
{
	uint32_t seq, dst_mmsi;
    unsigned int i, pos = 72;

	printf(" buflen %u pos+32: 72", bufferlen);
	for (i = 0; i < 4 && pos <= bufferlen; pos += 32) {
		dst_mmsi = protodec_henten(pos, 32, d->rbuffer);
        seq = (dst_mmsi & 0x03);
        dst_mmsi >>= 3;
		printf(" ack %u (to %08u seq %u)", i+1, dst_mmsi, seq);
		i++;
	}
}

/*
 *	8: Binary broadcast
 */

void protodec_8(struct demod_state_t *d, unsigned int bufferlen, time_t received_t, uint32_t mmsi)
{
	int appid_dac = protodec_henten(40, 10, d->rbuffer);
	int appid_fi = protodec_henten(50, 6, d->rbuffer);

	printf(" appid_dac %d appid_fi %d", appid_dac, appid_fi);
	if (appid_dac == 1) {
		printf("(%s)", appid_ifm(appid_fi));
		protodec_msg_bin(d->rbuffer, bufferlen, appid_fi, 56, received_t, mmsi);
	}
}

void protodec_18(struct demod_state_t *d, unsigned int bufferlen, time_t received_t, uint32_t mmsi)
{
	int longitude, latitude;
	float longit, latit;
	unsigned short course, sog, heading;

	longitude = protodec_henten(57, 28, d->rbuffer);
	if (((longitude >> 27) & 1) == 1)
		longitude |= 0xF0000000;
	longit = ((float) longitude) * 0.0000005f;

	latitude = protodec_henten(85, 27, d->rbuffer);
	if (((latitude >> 26) & 1) == 1)
		latitude |= 0xf8000000;
	latit = ((float) latitude) * 0.0000005f;

	course = protodec_henten(112, 12, d->rbuffer);
	sog = protodec_henten(46, 10, d->rbuffer);
	heading = protodec_henten(124, 9, d->rbuffer);

	printf(" lat %.6f lon %.6f course %.0f speed %.1f heading %d",
		   latit / 3.0f, longit / 3.0f, (float) course * 0.1f, (float) sog * 0.1f, heading);

	update_range(d, mmsi, latit / 3.0, longit / 3.0);
}

void protodec_19(struct demod_state_t *d, unsigned int bufferlen, time_t received_t, uint32_t mmsi)
{
	int pos, k;
	unsigned int A, B;
	unsigned char C, D;
	unsigned int shiptype;
	int letter;
	char name[21];
	/*
	 * Class B does not have destination, use "CLASS B" instead
	 * (same as ShipPlotter)
	 */

	/* get name */
	pos = 143;
	for (k = 0; k < 20; k++) {
		letter = protodec_henten(pos, 6, d->rbuffer);
		protodec_decode_sixbit_ascii(letter, name, k);
		pos += 6;
	}
	name[20] = 0;
	remove_trailing_spaces(name, 20);
	//printf("Name: '%s'\n", name);

	/* type of ship and cargo */
	shiptype = protodec_henten(263, 8, d->rbuffer);

	/* dimensions and reference GPS position */
	A = protodec_henten(271, 9, d->rbuffer);
	B = protodec_henten(271 + 9, 9, d->rbuffer);
	C = protodec_henten(271 + 9 + 9, 6, d->rbuffer);
	D = protodec_henten(271 + 9 + 9 + 6, 6, d->rbuffer);

	// printf("Length: %d\nWidth: %d\n",A+B,C+D);
	//printf("%09ld %d %d %f", mmsi, A + B, C + D);
	printf("CLASS B: name \"%s\" type %d length %d  width %d", name, shiptype, A+B, C+D);
}

void protodec_20(struct demod_state_t *d, unsigned int bufferlen)
{
	int ofs, slots, timeout, incr;
    unsigned int i, pos = 40;

	for (i = 0; i < 4 && pos + 30 < bufferlen; pos += 30) {
		ofs = protodec_henten(pos, 12, d->rbuffer);
		slots = protodec_henten(pos + 12, 4, d->rbuffer);
		timeout = protodec_henten(pos + 12 + 4, 3, d->rbuffer);
		incr = protodec_henten(pos + 12 + 4 + 3, 11, d->rbuffer);
		printf(" reserve %d (ofs %d slots %d timeout %d incr %d)",
			   i+1, ofs, slots, timeout, incr);
		i++;
	}
}

void protodec_24(struct demod_state_t *d, unsigned int bufferlen, time_t received_t, uint32_t mmsi)
{
	int partnr;
	int pos;
	int k, letter;
	unsigned int A, B;
	unsigned char C, D;
	unsigned int shiptype;
	char name[21];
	char callsign[7];
	/*
	 * Class B does not have destination, use "CLASS B" instead
	 * (same as ShipPlotter)
	 */

	/* resolve type 24 frame's part A or B */
	partnr = protodec_henten(38, 2, d->rbuffer);

	//printf("(partnr %d type %d): ",partnr, type);
	if (partnr == 0) {
		//printf("(Now in name:partnr %d type %d): ",partnr, type);
		/* get name */
		pos = 40;
		for (k = 0; k < 20; k++) {
			letter = protodec_henten(pos, 6, d->rbuffer);
			protodec_decode_sixbit_ascii(letter, name, k);
			pos += 6;
		}

		name[20] = 0;
		remove_trailing_spaces(name, 20);
		printf(" name \"%s\"", name);
	}

	if (partnr == 1) {
		//printf("(Now in data:partnr %d type %d): ",partnr, type);
		/* get callsign */
		pos = 90;
		for (k = 0; k < 6; k++) {
			letter = protodec_henten(pos, 6, d->rbuffer);
			protodec_decode_sixbit_ascii(letter, callsign, k);
			pos += 6;
		}
		callsign[6] = 0;
		remove_trailing_spaces(callsign, 6);

		/* type of ship and cargo */
		shiptype = protodec_henten(40, 8, d->rbuffer);

		/* dimensions and reference GPS position */
		A = protodec_henten(132, 9, d->rbuffer);
		B = protodec_henten(132 + 9, 9, d->rbuffer);
		C = protodec_henten(132 + 9 + 9, 6, d->rbuffer);
		D = protodec_henten(132 + 9 + 9 + 6, 6, d->rbuffer);
		printf(" callsign \"%s\" type %d length %d width %d", callsign, shiptype, A+B, C+D);
	}
}

#ifdef DEBUG_NMEA
#define NMEA_DBG(x) x
#else
#define NMEA_DBG(x)
#endif

void protodec_generate_nmea(struct demod_state_t *d, unsigned int bufferlen, unsigned int fillbits, time_t received_t)
{
	//6bits to nmea-ascii. One sentence len max 82char
	//inc. head + tail.This makes inside datamax 62char multipart, 62 single
	unsigned int senlen = 61; //this is normally not needed.For testing only. May be fixed number
    unsigned int k, pos = 0, m = 0, serbuffer_l;
	int letter;
	unsigned char sentences, sentencenum, nmeachk;

	if (bufferlen <= (senlen * 6)) {
		sentences = 1;
	} else {
		sentences = bufferlen / (senlen * 6);
		//sentences , if overflow put one more
		if (bufferlen % (senlen * 6) != 0)
			sentences++;
	};
	NMEA_DBG(printf("NMEA: %d sentences with max data of %d ascii chrs\n", sentences, senlen));
	sentencenum = 0;
	do {
		k = 13;		//leave room for nmea header
		while (k < senlen + 13 && bufferlen > pos) {
			letter = protodec_henten(pos, 6, d->rbuffer);
			// 6bit-to-ascii conversion by IEC
			if (letter < 40)
				letter = letter + 48;
			else
				letter = letter + 56;
			d->nmea_buffer[k] = letter;
			pos += 6;
			k++;
		}
		NMEA_DBG(printf("NMEA: Drop from loop with k:%d pos:%d senlen:%d bufferlen\n",
			            k, pos, senlen, bufferlen));
		//set nmea trailer with 00 checksum (calculate later)
		d->nmea_buffer[k] = 44;
		d->nmea_buffer[k + 1] = 48;
		d->nmea_buffer[k + 2] = 42;
		d->nmea_buffer[k + 3] = 48;
		d->nmea_buffer[k + 4] = 48;
		d->nmea_buffer[k + 5] = 0;
		sentencenum++;

		// printout one frame starts here
		//AIVDM,x,x,,, - header comes here first

		d->nmea_buffer[0] = 65;
		d->nmea_buffer[1] = 73;
		d->nmea_buffer[2] = 86;
		d->nmea_buffer[3] = 68;
		d->nmea_buffer[4] = 77;
		d->nmea_buffer[5] = 44;
		d->nmea_buffer[6] = 48 + sentences;
		d->nmea_buffer[7] = 44;
		d->nmea_buffer[8] = 48 + sentencenum;
		d->nmea_buffer[9] = 44;

		//if multipart message it needs sequential id number
		if (sentences > 1) {
			NMEA_DBG(printf("NMEA: It is multipart (%d/%d), add sequence number (%d) to header\n",
				            sentences, sentencenum, d->seqnr));
			d->nmea_buffer[10] = d->seqnr + 48;
			d->nmea_buffer[11] = 44;
			d->nmea_buffer[12] = 44;
			//and if the last of multipart we need to show fillbits at trailer
			if (sentencenum == sentences) {
				NMEA_DBG(printf("NMEA: It is last of multipart (%d/%d), add fillbits (%d) to trailer\n",
					            sentences, sentencenum, fillbits));
				d->nmea_buffer[k + 1] = 48 + fillbits;
			}
		} else {	//else put channel A & no seqnr to keep equal lenght (foo!)
			d->nmea_buffer[10] = 44;
			d->nmea_buffer[11] = 65;
			d->nmea_buffer[12] = 44;
		}

		//strcpy(nmea,"!AIVDM,1,1,,,");
		//calculate xor checksum in hex for nmea[0] until nmea[m]='*'(42)
		nmeachk = d->nmea_buffer[0];
		while (d->nmea_buffer[m] != 42) {	//!="*"
			nmeachk = nmeachk ^ d->nmea_buffer[m];
			m++;
		}

		// convert calculated checksum to 2 digit hex there are 00 as base
		// so if only 1 digit put it to later position to get 0-header 01,02...
		if (nmeachk <= 0x0F) {
			d->nmea_buffer[k + 4] = hex[nmeachk & 0x0f];
		} else {
			d->nmea_buffer[k + 3] = hex[((nmeachk >> 4) & 0x0f)];
			d->nmea_buffer[k + 4] = hex[((nmeachk >> 0) & 0x0f)];
		}
		//In final. Add header "!" and trailer <lf>
		// here it could be sent to /dev/ttySx
        d->nmea_buffer[k + 5] = '\n';
        d->nmea_buffer[k + 6] = '\0';
		serbuffer_l = strlen(d->nmea_buffer);
        if (d->nmea_out_fd != -1)
            write(d->nmea_out_fd, d->nmea_buffer, serbuffer_l);
		NMEA_DBG(printf("NMEA: End of nmea->ascii-loop with sentences:%d sentencenum:%d\n", sentences, sentencenum));
	} while (sentencenum < sentences);
}

void protodec_getdata(unsigned int bufferlen, struct demod_state_t *d)
{
	unsigned char type = protodec_henten(0, 6, d->rbuffer);
	uint32_t mmsi = protodec_henten(8, 30, d->rbuffer);
	unsigned int k, fillbits = 0;
	time_t received_t;
	time(&received_t);

	if (type > MAX_AIS_PACKET_TYPE /* 9 */)
		return;

	DBG(printf("Bufferlen: %d,", bufferlen));

	if (bufferlen % 6 > 0) {
		fillbits = 6 - (bufferlen % 6);
		for (k = bufferlen; k < bufferlen + fillbits; k++)
			d->rbuffer[k] = 0;

		bufferlen = bufferlen + fillbits;
	}

	DBG(printf(" fixed Bufferlen: %d with %d fillbits\n", bufferlen, fillbits));

	/* generate an NMEA string out of the binary packet */
	protodec_generate_nmea(d, bufferlen, fillbits, received_t);

	//multipart message ready. Increase seqnr for next one
	//rolling 1-9. Single msg ready may also increase this, no matter.
	d->seqnr++;
	if (d->seqnr > 9)
		d->seqnr = 0;

	printf("type %u mmsi %08u:", type, mmsi);

	switch (type) {
	case 1: /* position packets */
	case 2:
	case 3:
		protodec_pos(d, bufferlen, received_t, mmsi);
		break;

	case 4: /* base station position */
		protodec_4(d, bufferlen, received_t, mmsi);
		break;

	case 5: /* vessel info */
		protodec_5(d, bufferlen, received_t, mmsi);
		break;

	case 6: /* Addressed binary message */
		protodec_6(d, bufferlen, received_t, mmsi);
		break;

	case 7: /* Binary acknowledge */
	case 13: /* Safety related acknowledge */
		protodec_7_13(d, bufferlen, received_t, mmsi);
		break;

	case 8: /* Binary broadcast message */
		protodec_8(d, bufferlen, received_t, mmsi);
		break;

	case 18: /* class B transmitter position report */
		protodec_18(d, bufferlen, received_t, mmsi);
		break;

	case 19: /* class B transmitter vessel info */
		protodec_19(d, bufferlen, received_t, mmsi);
		break;

	case 24: /* class B transmitter info */
		protodec_24(d, bufferlen, received_t, mmsi);
		break;

	case 20:
		protodec_20(d, bufferlen);
		break;

	default:
		break;
	}

	printf(" (!%s)\n", d->nmea_buffer);
}

void protodec_decode(unsigned char *in, unsigned int count, struct demod_state_t *d)
{
	unsigned int i = 0, bufferlength, correct;

	while (i < count) {
		switch (d->state) {
		case ST_DATA:
			if (d->bitstuff) {
				if (in[i] == 1) {
					d->state = ST_STOPSIGN;
					d->ndata = 0;
					DBG(printf("%d", in[i]));
					d->bitstuff = 0;
				} else {
					d->ndata++;
					d->last = in[i];
					d->bitstuff = 0;
				}
			} else {
				if (in[i] == d->last && in[i] == 1) {
					d->antallenner++;
					if (d->antallenner == 4) {
						d->bitstuff = 1;
						d->antallenner = 0;
					}
				} else {
					d->antallenner = 0;
                }

				DBG(printf("%d", in[i]));

				d->buffer[d->bufferpos] = in[i];
				d->bufferpos++;
				d->ndata++;

				if (d->bufferpos >= 961) {
					protodec_reset(d);
				}
			}
			break;

		case ST_SKURR:	// The state when no reasonable input is coming
			if (in[i] != d->last)
				d->antallpreamble++;
			else
				d->antallpreamble = 0;
			d->last = in[i];
			if (d->antallpreamble > 14 && in[i] == 0) {
				d->state = ST_PREAMBLE;
				d->nskurr = 0;
				d->antallpreamble = 0;
				DBG(printf("Preamble\n"));
			}
			d->nskurr++;
			break;

		case ST_PREAMBLE:	// Switches to this state when preamble has been discovered
			DBG(printf("..%d..", in[i]));
			if (in[i] != d->last && d->nstartsign == 0) {
				d->antallpreamble++;
			} else {
				if (in[i] == 1)	{ //To ettal har kommet etter hverandre
					if (d->nstartsign == 0) {	// Forste gang det skjer
						d->nstartsign = 3;
						d->last = in[i];
					} else if (d->nstartsign == 5) {	// Har oppdaget start av startsymbol
						d->nstartsign++;
						d->npreamble = 0;
						d->antallpreamble = 0;
						d->state = ST_STARTSIGN;
					} else {
						d->nstartsign++;
					}
				} else { //To nuller har kommet etter hverandre
					if (d->nstartsign == 0) {
						d->nstartsign = 1;
					} else {
						protodec_reset(d);
					}
				}
			}
			d->npreamble++;
			break;

		case ST_STARTSIGN:
			//printf("..%d..",in[i]);
			//printf("Startsign: %d\n",d->nstartsign);
			if (d->nstartsign >= 7) {
				if (in[i] == 0) {
					DBG(printf("\nData:\n"));
					d->state = ST_DATA;
					d->nstartsign = 0;
					d->antallenner = 0;
					memset(d->buffer, 0, DEMOD_BUFFER_LEN);
					d->bufferpos = 0;
				} else {
					protodec_reset(d);
				}
			} else if (in[i] == 0) {
				protodec_reset(d);
			}
			d->nstartsign++;
			break;

		case ST_STOPSIGN:
			bufferlength = d->bufferpos - 6 - 16;
			if (in[i] == 0 && bufferlength > 0) {
				DBG(printf("%d\n\nFrame received OK. %u bits\n", in[i], bufferlength));
				correct = protodec_calculate_crc(bufferlength, d);
				if (correct) {
					DBG(printf("CRC Checksum correct!\n"));
					d->receivedframes++;
				} else {
					DBG(printf("CRC Checksum incorrect!!!\n"));
					d->lostframes++;
				}
				protodec_getdata(bufferlength, d);
			} else {
				DBG(printf("\n\nERROR in Frame\n"));
				d->lostframes2++;
			}
			DBG(printf("_________________________________________________________\n\n"));
			protodec_reset(d);
			break;
		}
		d->last = in[i];
		i++;
	}
}

