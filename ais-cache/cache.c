
/*
 *	cache.c
 *
 *	(c) Heikki Hannikainen 2008
 *
 *	Cache received AIS position reports, storing the most current
 *	up-to-date information for each MMSI, so that it can be sent out
 *	at regular intervals.
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
#include "cache.h"
#include "splay.h"

//#define DEBUG_CACHE
#ifdef DEBUG_CACHE
#define CACHE_DBG(x) x
#else
#define CACHE_DBG(x)
#endif

/* are we caching positions? */
int cache_positions = 0;

/* the splay tree root for the cache */
struct sptree *cache_spt;

char *estrdup(const char *str)
{
    unsigned int len = strlen(str);
    char *copy = malloc(len+1);
    memcpy(copy, str, len);
    copy[len] = '\0';
    return copy;
}

/*
 *	initialize the cache
 */

int cache_init(void)
{
	cache_spt = sp_init();
	cache_spt->symbols = NULL;

	/* ok, we're clear to go */
	cache_positions = 1;
	return 0;
}

/*
 *	free a cache entry
 */

void cache_free_entry(struct cache_ent *e)
{
	if (e->name)
		free(e->name);
	if (e->destination)
		free(e->destination);
	if (e->callsign)
		free(e->callsign);
	free(e);
}

/*
 *	uninitialize a copy of the cache
 */

int cache_free(struct sptree *sp)
{
	struct spblk *x, *nextx;
	struct cache_ent *e;
	int freed = 0;

	for (x = sp_fhead(sp); x != NULL; x = nextx) {
		nextx = sp_fnext(x);
		e = (struct cache_ent *)x->data;
		cache_free_entry(e);
		sp_delete(x, sp);
		freed++;
	}

	return 0;
}

/*
 *	uninitialize the main cache
 */

int cache_deinit(void)
{
	int ret = cache_free(cache_spt);
	free(cache_spt);
	sp_free_freelist();
	return ret;
}

/*
 *	get the existing cache (for export) and create a new one
 */

struct sptree *cache_rotate(void)
{
	struct sptree *got_spt = cache_spt;
	cache_spt = sp_init();
	cache_spt->symbols = NULL;
	return got_spt;
}

/*
 *	look up an existing entry - if it doesn't exist, create one
 */

static struct cache_ent *cache_get(unsigned int mmsi)
{
	struct spblk *spl;
	struct cache_ent *e;

	/* check if we already have this key */
	spl = sp_lookup((spkey_t)mmsi, cache_spt);

	if (spl) {
		e = (struct cache_ent *)spl->data;
		CACHE_DBG(printf("cache_get hit: %d", mmsi));
	} else {
		CACHE_DBG(printf("cache_get miss: %d", mmsi));
		/* oh, new ship... install in cache */
		spl = sp_install((spkey_t)mmsi, cache_spt);
		e = malloc(sizeof(*e));
		spl->data = (void *)e;

		/* reset data in the allocated entry */
		memset((void *)e, 0, sizeof(*e));

		/* floats need to be set separately */
		e->lat = 0;
		e->lon = 0;
		e->hdg = -1;
		e->course = -1;
		e->sog = -1;
		e->shiptype = -1;
		e->imo = -1;
		e->navstat = -1;
		e->A = e->B = e->C = e->D = -1;
		e->persons_on_board = -1;
	}

	return e;
}

/*
 *	cache a ship's position
 */

int cache_position(int received_t, unsigned int mmsi, int navstat, float lat, float lon,
                   int hdg, float course, int rateofturn, float sog)
{
	struct cache_ent *e;

	CACHE_DBG(printf("cache_position %d t %d", mmsi, received_t));

	e = cache_get(mmsi);

	e->mmsi = mmsi;
	e->received_pos = received_t;
	e->mmsi = mmsi;
	e->lat = lat;
	e->lon = lon;
	e->hdg = hdg;
	e->course = course;
	e->sog = sog;
	e->navstat = navstat;
	return 0;
}

/*
 *	cache static vessel data
 */

int cache_vesseldata(int received_t, unsigned int mmsi, unsigned int imo, char *callsign,
                     char *name, char *destination, unsigned int shiptype,
                     unsigned int A, unsigned int B, unsigned char C, unsigned char D, float draught)
{
	struct cache_ent *e;

	CACHE_DBG(printf("cache_vesseldata %d: name '%s' dest '%s' t %d", mmsi, name, destination, received_t));

	e = cache_get(mmsi);

	e->mmsi = mmsi;
	e->imo = imo;
	e->received_data = received_t;
	if (!e->callsign || strcmp(e->callsign, callsign) != 0) {
		if (e->callsign)
			free(e->callsign);
		e->callsign = estrdup(callsign);
	}
	if (!e->name || strcmp(e->name, name) != 0) {
		if (e->name)
			free(e->name);
		e->name = estrdup(name);
	}
	if (!e->destination || strcmp(e->destination, destination)) {
		if (e->destination)
			free(e->destination);
		e->destination = estrdup(destination);
	}
	e->shiptype = shiptype;
	e->A = A;
	e->B = B;
	e->C = C;
	e->D = D;
	e->draught = draught;
	return 0;
}

int cache_vesseldatab(int received_t, unsigned int mmsi, char *callsign,
                      unsigned int shiptype, unsigned int A, unsigned int B, unsigned char C, unsigned char D)
{
	struct cache_ent *e;

	CACHE_DBG(printf("cache_vesseldatab %d:  t %d", mmsi, received_t));

	e = cache_get(mmsi);

	e->mmsi = mmsi;
	e->imo = 0; //NO imo;
	e->received_data = received_t;
	if (!e->callsign || strcmp(e->callsign, callsign) != 0) {
		if (e->callsign)
			free(e->callsign);
		e->callsign = estrdup(callsign);
	}

	e->shiptype = shiptype;
	e->A = A;
	e->B = B;
	e->C = C;
	e->D = D;
	e->draught = 0;//NO draught;
	return 0;
}

int cache_vesseldatabb(int received_t, unsigned int mmsi, unsigned int shiptype,
                       unsigned int A, unsigned int B, unsigned char C, unsigned char D)
{
	struct cache_ent *e;

	CACHE_DBG(printf("cache_vesseldatab %d:  t %d", mmsi, received_t));

	e = cache_get(mmsi);
	e->mmsi = mmsi;
	e->imo = 0; //NO imo;
	e->received_data = received_t;
	e->shiptype = shiptype;
	e->A = A;
	e->B = B;
	e->C = C;
	e->D = D;
	e->draught = 0;//NO draught;
	return 0;
}

int cache_vesselname(int received_t, unsigned int mmsi, char *name, const char *destination)
{
	struct cache_ent *e;

	CACHE_DBG(printf("cache_vesselname %d: name '%s' destination '%s'  t %d", mmsi, name, destination, received_t));

	e = cache_get(mmsi);
	e->mmsi = mmsi;
	e->received_data = received_t;

	if (!e->name || strcmp(e->name, name) != 0) {
		if (e->name)
			free(e->name);
		e->name = estrdup(name);
	}

	if (!e->destination || strcmp(e->destination, destination)) {
		if (e->destination)
			free(e->destination);
		e->destination = estrdup(destination);
	}

	return 0;
}

int cache_vessel_persons(int received_t, unsigned int mmsi, unsigned int persons_on_board)
{
	struct cache_ent *e;

	CACHE_DBG(printf("cache_vessel_persons %d: %d t %d", mmsi, persons_on_board, received_t));

	e = cache_get(mmsi);
	e->mmsi = mmsi;
	e->received_persons_on_board = received_t;
	e->persons_on_board = persons_on_board;
	return 0;
}

