
/*
 *	out_json.c
 *
 *	(c) Heikki Hannikainen 2008
 *
 *	Send ship position data out in the JSON AIS format:
 *	http://wiki.ham.fi/JSON_AIS.en
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
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include "splay.h"
#include "cache.h"
#include "protodec.h"
#define TBUF_LEN 33

/*
 *  Append a formatted string to a dynamically allocated string
 */

char *str_append(char *s, const char *fmt, ...)
{
    va_list args;
    char buf[1024];
    size_t len, len2;
    char *ret;

    va_start(args, fmt);
    len2 = vsnprintf(buf, 1023, fmt, args);
    va_end(args);
    buf[1023] = 0; /* maybe not necessary... paranoid? */

    if (s)
        len = strlen(s);
    else
        len = 0;

    ret = realloc(s, len + len2 + 1);
    memcpy(ret + len, buf, len2);
    ret[len+len2] = '\0';
    return ret;
}

/*
 *	Encode an unix timestamp in JSON AIS format
 *
 *	YYYYMMDDHHMMSS
 *	01234567890123
 */

int time_jsonais(time_t *t, char *buf, unsigned int buflen)
{
	int i;
	struct tm dt;

	/* check that the buffer is large enough - we use
	 * 14 bytes plus the NULL
	 */
	if (buflen < 21) {
		printf("time_jsonais: not enough space to produce JSON AIS timestamp");
		return -1;
	}

	/* thread-safe UTC */
	if (gmtime_r(t, &dt) == NULL) {
		printf("time_jsonais: gmtime_r failed");
		return -1;
	}

	i = snprintf(buf, buflen, "%04d%02d%02d %02u:%02u:%02u",
                 dt.tm_year + 1900, dt.tm_mon + 1, dt.tm_mday,
                 dt.tm_hour, dt.tm_min, dt.tm_sec);
	//printf("time_jsonais: %d => %s", *t, buf);
	return i;
}

/*
 *	export the contents of the buffer splaytree
 */

char *jsonout_export(void)
{
	unsigned int entries = 0;
	unsigned int exported = 0;
	struct spblk *x, *nextx;
	struct sptree *sp;
	struct cache_ent *e;
	char *json = NULL;
	char tbuf[TBUF_LEN];
	time_t now;

	time(&now);
	time_jsonais(&now, tbuf, TBUF_LEN);

	/* fill in initial json */
	json = str_append(json,
		"{\n"
		"\t\"protocol\": \"jsonais\",\n"
		"\t\"encodetime\": \"%s\",\n"
		"\t\"groups\": [\n" /* start of groups */
		"\t\t{\n" /* start of group */
		"\t\t\t\"path\": [ { \"name\": \"asdf\" } ],\n"
		"\t\t\t\"msgs\": [\n", tbuf);

	/* get the current position cache */
	sp = cache_rotate();

	/* expire old entries */
	for (x = sp_fhead(sp); x != NULL; x = nextx) {
		entries++;
		nextx = sp_fnext(x);
		e = (struct cache_ent *)x->data;
		unsigned int got_pos = ((e->lat > 0.0001f || e->lat < -0.0001f) && (e->lon > 0.0001f || e->lon < -0.0001f));

		if ((e->mmsi) && (got_pos)) {
			printf("jsonout: exporting MMSI %d position", e->mmsi);
			time_jsonais(&e->received_pos, tbuf, TBUF_LEN);
			json = str_append(json, "%s{\"msgtype\": 3, \"mmsi\": %d, \"rxtime\": \"%s\"",
				              (exported == 0) ? "" : ",\n", e->mmsi, tbuf);

			json = str_append(json, ", \"lat\": %.7f, \"lon\": %.7f", e->lat, e->lon);
			if (e->course >= 0)
				json = str_append(json, ", \"course\": %.1f", e->course);
			if (e->hdg >= 0)
				json = str_append(json, ", \"heading\": %d", e->hdg);
			if (e->sog >= 0)
				json = str_append(json, ", \"speed\": %.1f", e->sog);
			if (e->navstat >= 0)
				json = str_append(json, ", \"status\": %d", e->navstat);

			json = str_append(json, "}");
			exported++;
		}

		if ((e->mmsi) && (e->name)) {
			printf("jsonout: exporting MMSI %d data", e->mmsi);
			time_jsonais(&e->received_data, tbuf, TBUF_LEN);
			json = str_append(json, "%s{\"msgtype\": 5, \"mmsi\": %d, \"rxtime\": \"%s\"",
				              (exported == 0) ? "" : ",\n", e->mmsi, tbuf);

			if (e->imo >= 0)
				json = str_append(json, ", \"imo\": %d", e->imo);
			if (e->shiptype >= 0)
				json = str_append(json, ", \"shiptype\": %d", e->shiptype);
			if (e->callsign)
				json = str_append(json, ", \"callsign\": \"%s\"", e->callsign);
			if (e->name)
				json = str_append(json, ", \"shipname\": \"%s\"", e->name);
			if (e->destination)
				json = str_append(json, ", \"destination\": \"%s\"", e->destination);

			if (e->A >= 0 && e->B >= 0) {
				json = str_append(json, ", \"length\": %d", e->A + e->B);
				json = str_append(json, ", \"ref_front\": %d", e->A);
			}

			if (e->draught >= 0)
				json = str_append(json, ", \"draught\": %.1f", e->draught);

			if (e->C >= 0 && e->D >= 0) {
				json = str_append(json, ", \"width\": %d", e->C + e->D);
				json = str_append(json, ", \"ref_left\": %d", e->C);
			}

			json = str_append(json, "}");
			exported++;
		}

		if (e->persons_on_board >= 0) {
			printf("jsonout: exporting MMSI %d persons_on_board %d", e->mmsi, e->persons_on_board);
			time_jsonais(&e->received_persons_on_board, tbuf, TBUF_LEN);
			json = str_append(json, "%s{\"msgtype\": 8, \"mmsi\": %d, \"persons_on_board\": %d, \"rxtime\": \"%s\"}",
				              (exported == 0) ? "" : ",\n", e->mmsi, e->persons_on_board, tbuf);
			exported++;
		}

		cache_free_entry(e);
		sp_delete(x, sp);
	}

	json = str_append(json,
		"\n\n"
		"\t\t\t]\n" /* end of message array */
		"\t\t}\n" /* end of the message group */
		"\t]\n" /* end of groups */
		"}\n" /* end of the whole json blob */
		);

	/* clean up */
	if (sp) {
		sp_null(sp);
		free(sp);
	}

	printf("jsonout: %s", json);
    return json;
}

