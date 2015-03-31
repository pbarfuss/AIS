#include <stdint.h>
#include <stdio.h>

void generate_crc16_table(uint16_t *ctx, uint32_t poly){
    uint32_t i, j, c;

    for (i = 0; i < 256; i++) {
        for (c = i << 24, j = 0; j < 8; j++)
            c = (c<<1) ^ ((poly<<16) & (((int32_t)c)>>31));
        ctx[i] = (c >> 16);
    }
}

static void write_u16_array(const char *name, const uint16_t *data, unsigned int len)
{
    unsigned int i;
    printf("static uint16_t %s[%u] = {\n", name, len);
    printf("   ");
    for (i = 0; i < len - 1; i++) {
       printf("0x%04x, ", data[i]);
       if ((i & 7) == 7) printf("\n   ");
    }
    printf("0x%04x\n};\n\n", data[i]);
}

int main(void) {
    uint16_t crctab[257];
    generate_crc16_table(crctab, 0x8408);
    write_u16_array("crc16tab", crctab, 256);
    return 0;
}

