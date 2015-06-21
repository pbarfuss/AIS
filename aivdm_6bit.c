#include <stdint.h>
#include <string.h>

static unsigned char convtab[] = {"0123456789:;<=>?@ABCDEFGHIJKLMNOPQRSTUVW`abcdefghijklmnopqrstuvw"};

/* The "I-can't-believe-it's-not-base64" transform. */
unsigned int ais_binary_to_ascii(unsigned char *outbits, unsigned char *bits, unsigned int len)
{
    unsigned int l;
    for (l=0;l<len;l+=6) {
        outbits[l/6] = convtab[bits[l/6] & 0x3f];
    }
    outbits[len] = '\0';
    return ((len-5)/6);
}

void ais_ascii_to_ascii(unsigned char *bitvec, unsigned int start, unsigned int count, char *to)
{
    unsigned int i;

    for (i = 0; i < count; i++) {
        char eightbit, sixbit = ubits(bitvec, start + 6*i, 6, 0);
	    if (sixbit >= 1 && sixbit < 32) {
		    eightbit = sixbit + 64;
	    } else if (sixbit < 64) {
		    eightbit = sixbit;
	    } else {
            eightbit = ' ';
        }
        to[i] = eightbit;
    }
    to[i] = '\0';
}

/*
 * this checksum is trivially linear, so use this to not recalc the sum of "AIVDM,bla,bla,bla" repeatedly.
 * Initialize to: checksum of "AIVDM" -> 87
 * (We actually want 86 for most messages because we can cheat harder: most common messages in AIS have the following props:
 * 1-4 are all always 1 fragment count, 1 fragment number. Those cancel.
 * Additionally, they're never multipart, and always align on 6 bits correctly after the I-can't-believe-it's-not-base64 transform.
 * That's where our additional 1 bit comes from.
 * This means for these messages all we need to do is compute the checksum on the message, then xor it with 'A' or 'B'
 * (for the channel indication) and we're done. Messages we can always do this for, that I know of so far:
 * 1, 2, 3, (Pos'n Report, some ~80% of all AIS messages) 4, (Base Station Report) 11, (Timeinfo/data Report) 21 (AtoN Message).
 *
 */
unsigned int aivdm_checksum(unsigned int sum, unsigned char *msg, unsigned int len)
{
    unsigned int i;
    for(i = 0; i < len; i++) {
        sum ^= msg[i];
    }
    return sum;
}

