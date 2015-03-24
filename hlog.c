/*
 *	(c) Heikki Hannikainen, OH7LZB <hessu@hes.iki.fi>
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
 *	
 */

/*
 *	hlog.c
 *
 *	logging facility with configurable log levels and
 *	logging destinations
 */

#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <errno.h>
#include "hlog.h"

int log_dest = L_DEFDEST;	/* Logging destination */
int log_level = LOG_INFO;	/* Logging level */
int log_facility = LOG_LOCAL1;	/* Logging facility */
char *log_name = NULL;		/* Logging name */

char log_basename[] = "gnuais.log";
char *log_dir = NULL;		/* Access log directory */
char *log_fname = NULL;		/* Access log file name */
int log_file = -1;		/* If logging to a file, the file name */

char *log_levelnames[] = {
	"EMERG",
	"ALERT",
	"CRIT",
	"ERR",
	"WARNING",
	"NOTICE",
	"INFO",
	"DEBUG",
	NULL
};

char *log_destnames[] = {
	"none",
	"stderr",
	"syslog",
	"file",
	NULL
};

/*
 *	Append a formatted string to a dynamically allocated string
 */

char *str_append(char *s, const char *fmt, ...)
{
	va_list args;
	char buf[LOG_LEN];
	size_t len, len2;
	char *ret;
	
	va_start(args, fmt);
	len2 = vsnprintf(buf, LOG_LEN, fmt, args);
	va_end(args);
	buf[LOG_LEN-1] = 0; /* maybe not necessary... paranoid? */
	
	if (s)
		len = strlen(s);
	else
		len = 0;

	ret = realloc(s, len + len2 + 1);
    memcpy(ret + len, buf, len2);
    ret[len+len2] = '\0';
	return ret;
}

unsigned int itoa10(char *__restrict bufend, unsigned int uval)
{
    unsigned int digit;
    unsigned int i = 0, j = 0;
    unsigned char _buf[24];
    do {
        digit = uval % 10;
        uval /= 10;
        _buf[i++] = (digit + '0');
    } while (uval);
    while(i-- > 0) { bufend[j++] = _buf[i]; }
    bufend[j] = '\0';
    return j;
}

/*
 *	Pick a log level
 */

int pick_loglevel(char *s, char **names)
{
	int i;
	
	for (i = 0; (names[i]); i++)
		if (!strcasecmp(s, names[i]))
			return i;
			
	return -1;
}

void set_logname(char *name)
{
	if (log_name)
		free(log_name);
		
	if (!(log_name = hstrdup(name))) {
		fprintf(stderr, "logger: out of memory!\n");
		exit(1);
	}
}

/*
 *	Open log
 */
 
int open_log(char *name, int reopen)
{
	if (log_dest == L_SYSLOG)
		openlog(name, LOG_NDELAY|LOG_PID, log_facility);
	
	if (log_dest == L_FILE) {
		if (log_fname)
			free(log_fname);
		
		log_fname = malloc(strlen(log_dir) + strlen(log_basename) + 2);
		sprintf(log_fname, "%s/%s", log_dir, log_basename);
		
		log_file = open(log_fname, O_WRONLY|O_CREAT|O_APPEND, S_IRUSR|S_IWUSR|S_IRGRP);
		if (log_file < 0) {
			fprintf(stderr, "logger: Could not open %s: %s\n", log_fname, strerror(errno));
            return -1;
		}
	}
	
	if (log_dest == L_FILE)
		hlog(LOG_DEBUG, "Log file %s %sopened on fd %d", log_fname, (reopen) ? "re" : "", log_file);
	
	return 0;
}

/*
 *	Close log
 */
 
int close_log(int reopen)
{
	if (log_name) {
		free(log_name);
		log_name = NULL;
	}
	
	if (log_dest == L_SYSLOG) {
		closelog();
	} else if (log_dest == L_FILE) {
		if (close(log_file)) {
			fprintf(stderr, "hemserv logger: Could not close log file %s: %s\n", log_fname, strerror(errno));
        }
		log_file = -1;
		free(log_fname);
		log_fname = NULL;
	}
	
	if (reopen) {
		open_log(log_fname, 1);
    } else {
		free(log_fname);
		log_fname = NULL;
    }
	
	return 0;
}

/*
 *	Log a message
 */

int hlog(int priority, const char *fmt, ...)
{
	va_list args;
	char s[LOG_LEN];
	char wb[LOG_LEN];
	int len, w;
	struct tm lt;
	struct timeval tv;
	
	if (priority > 7)
		priority = 7;
	else if (priority < 0)
		priority = 0;
	
	if (priority > log_level)
		return 0;
	
	va_start(args, fmt);
	vsnprintf(s, LOG_LEN, fmt, args);
	va_end(args);

	gettimeofday(&tv, NULL);
	gmtime_r(&tv.tv_sec, &lt);
	
	len = snprintf(wb, LOG_LEN-1, "%4d/%02d/%02d %02d:%02d:%02d.%06d %s[%u] %s: %s\n",
			       lt.tm_year + 1900, lt.tm_mon + 1, lt.tm_mday, lt.tm_hour, lt.tm_min, lt.tm_sec, (int)tv.tv_usec,
			       (log_name) ? log_name : "gnuais", getpid(), log_levelnames[priority], s);
	wb[LOG_LEN-1] = 0;
	if (log_dest == L_STDERR) {
        write(2, wb, len);
	} else if ((log_dest == L_FILE) && (log_file >= 0)) {
		if ((w = write(log_file, wb, len)) != len) {
			len = snprintf(wb, LOG_LEN-1, "logger: Could not write to %s (fd %d): %s\n", log_fname, log_file, strerror(errno));
            write(2, wb, len);
        }
	} else if (log_dest == L_SYSLOG) {
		syslog(priority, "%s: %s", log_levelnames[priority], s);
	}
	
	return 1;
}

/*
 *	Write my PID to file
 *	FIXME: add flock(TRY) to prevent multiple copies from running
 */

int writepid(const char *name)
{
    int fd = -1;
	char pidbuf[17];
    unsigned int pidlen;
	if ((fd = open(name, O_WRONLY|O_CREAT|O_TRUNC|O_EXCL, 0644)) < 0) {
		hlog(LOG_CRIT, "pid file %s: %s", name, strerror(errno));
        return 0;
	}
    pidlen = itoa10(pidbuf, getpid());
    write(fd, pidbuf, pidlen);
    close(fd);
    return 1;
}

