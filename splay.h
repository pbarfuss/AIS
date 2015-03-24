/*
** sptree.h:  The following type declarations provide the binary tree
**  representation of event-sets or priority queues needed by splay trees
**
**  assumes that data and datb will be provided by the application
**  to hold all application specific information
**
**  assumes that key will be provided by the application, comparable
**  with the compare function applied to the addresses of two keys.
*/

# ifndef SPTREE_H
# define SPTREE_H

#include <stdint.h>

typedef uint32_t	spkey_t;

struct spblk {
	struct spblk	*leftlink;
	struct spblk	*rightlink;
	struct spblk	*uplink;
	spkey_t		key;
	void		*data;
};

struct sptree {
	struct spblk	*root;		/* root node */
	struct sptree	*symbols;	/* If this db needs symbol support,
					   here is another sptree for those */
	long long	eltscnt;	/* How many elements in this tree */
	long long	lookups;	/* number of splookup()s */
	long long	lkpcmps;	/* number of lookup comparisons */
	long long	enqs;		/* number of spenq()s */
	long long	enqcmps;	/* compares in spenq */
	long long	splays;
	long long	splayloops;
};

extern long sp_allocated;
extern long sp_entries;

extern void sp_free_freelist(void);
extern struct sptree *sp_init(void); /* init tree */
extern struct spblk *sp_lookup(spkey_t key,
				   struct sptree *q);	/* find key in a tree*/
extern struct spblk *sp_install(spkey_t key, struct sptree *q); /* enter an item,
							   allocating or replacing */
extern void sp_scan(int (*f)(struct spblk *), struct spblk *n, struct sptree *q);	/* scan forward through tree */
extern void sp_delete(struct spblk *n, struct sptree *q); /* delete node from tree */
extern void        sp_null(struct sptree *);
extern const char *sp_stats(struct sptree *q);/* return tree statistics */
extern spkey_t     symbol(const void *s);	/* build this into a symbol */
extern spkey_t     symbol_lookup(const void *s);
extern spkey_t     symbol_db(const void *, struct sptree *);
extern spkey_t     symbol_lookup_db(const void *, struct sptree *);
extern spkey_t     symbol_db_mem(const void *, int, struct sptree *);
extern spkey_t symbol_lookup_db_mem(const void *, int, struct sptree *);
extern void	   symbol_free_db(const void *, struct sptree *);
extern void	   symbol_null_db(struct sptree *);

extern struct spblk *lookup_incoresp(const char *, struct sptree *);
extern int      add_incoresp(const char *, const char *, struct sptree *);
extern int     addd_incoresp(const char *, const void *, struct sptree *);

extern const char *pname(spkey_t id);

extern struct spblk * sp_fhead(struct sptree *); /* fast non-splaying head */
extern struct spblk * sp_fnext(struct spblk *); /* fast non-splaying next */

#endif	/* SPTREE_H */
