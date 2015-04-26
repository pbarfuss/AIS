/*
 * rtl-sdr, turns your Realtek RTL2832 based DVB dongle into a SDR receiver
 * Copyright (C) 2012-2014 by Steve Markgraf <steve@steve-m.de>
 * Copyright (C) 2012 by Dimitri Stolnikov <horiz0n@gmx.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <signal.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <errno.h>
#include "rtl-sdr.h"
#include "tuner_e4k.h"
#include "tuner_r82xx.h"
#define APPLY_PPM_CORR(val,ppm) (((val) * (1.0 + (ppm) * 1e-6)))
#define TWO_POW(n)		((float)(1ULL<<(n)))

#include <libusb.h>

/** In: device-to-host */
#define ENDPOINT_IN 0x80

/** Out: host-to-device */
#define ENDPOINT_OUT 0x00

typedef struct _usb_device_t {
    unsigned char busnum, devaddr, speed;
    unsigned short idVendor, idProduct;
    const char *name;
    char serial[256];
} usb_device_t;

/* we traverse usbfs without knowing how many devices we are going to find.
 * so we create this discovered_devs model which is similar to a linked-list
 * which grows when required. it can be freed once discovery has completed,
 * eliminating the need for a list node in the libusb_device structure
 * itself. */
struct discovered_devs {
    size_t len;
    size_t capacity;
    usb_device_t devices[0];
};

/*
 * All libusb callback functions should be marked with the LIBUSB_CALL macro
 * to ensure that they are compiled with the same calling convention as libusb.
 *
 * If the macro isn't available in older libusb versions, we simply define it.
 */
#ifndef LIBUSB_CALL
#define LIBUSB_CALL
#endif

/* libusb < 1.0.9 doesn't have libusb_handle_events_timeout_completed */
#ifndef HAVE_LIBUSB_HANDLE_EVENTS_TIMEOUT_COMPLETED
#define libusb_handle_events_timeout_completed(ctx, tv, c) \
	libusb_handle_events_timeout(ctx, tv)
#endif

typedef struct rtlsdr_tuner_iface {
	/* tuner interface */
	int (*init)(void *);
	int (*exit)(void *);
    int (*set_freq)(void *, uint32_t freq /* Hz */);
    int (*set_bw)(void *, uint32_t bw /* kHz */);
    int (*set_lna_gain)(void *, unsigned int gain /* half dB */);
    int (*set_mixer_gain)(void *, unsigned int gain /* half dB */);
    int (*set_gain_mode)(void *, uint8_t manual);
} rtlsdr_tuner_iface_t;

#define FIR_LEN 16

/*
 * FIR coefficients.
 *
 * The filter is running at XTal frequency. It is symmetric filter with 32
 * coefficients. Only first 16 coefficients are specified, the other 16
 * use the same values but in reversed order. The first coefficient in
 * the array is the outer one, the last, the last is the inner one.
 * First 8 coefficients are 8 bit signed integers, the next 8 coefficients
 * are 12 bit signed integers. All coefficients have the same weight.
 *
 * Default FIR coefficients used for DAB/FM by the Windows driver,
 * the DVB driver uses different ones
 */
static int fir_default[FIR_LEN] = {
	-54, -36, -41, -40, -32, -14, 14, 53,	/* 8 bit signed */
	101, 156, 215, 273, 327, 372, 404, 421	/* 12 bit signed */
};

struct rtlsdr_dev {
	libusb_context *ctx;
	struct libusb_device_handle *devh;
	/* rtl demod context */
	uint32_t rate; /* Hz */
	uint32_t rtl_xtal; /* Hz */
	/* tuner context */
	enum rtlsdr_tuner tuner_type;
	rtlsdr_tuner_iface_t *tuner;
	uint32_t tun_xtal; /* Hz */
	uint32_t freq; /* Hz */
	uint32_t offs_freq; /* Hz */
	int corr; /* ppm */
	uint32_t gain; /* tenth dB */
	struct e4k_state e4k_s;
	struct r82xx_priv r82xx_p;
	/* status */
	int dev_lost;
	int spectrum_inversion;
};

void rtlsdr_set_gpio_bit(rtlsdr_dev_t *dev, uint8_t gpio, int val);

static const uint8_t e4k_lna_mixer_gains[19*2] = {
	 0,  0,
	 5,  0,
	10,  0,
	15,  0,
	20,  0,
	25,  0,
    15, 15,
    20, 15,
    25, 15,
    30, 15,
    35, 15,
    40, 15,
    45, 15,
    50, 15,
    55, 15,
    60, 15,
    60, 15,
    60, 15,
    60, 15
};

static const uint8_t r82xx_lna_mixer_gains[19*2] = {
    0,  0,
    2,  2,
    5,  4,
   13,  2,
   13,  6,
   20,  6,
   23,  6,
   23, 10,
   29, 10,
   29, 14,
   39, 12,
   39, 16,
   45, 16,
   45, 20,
   50, 20,
   50, 24,
   57, 24,
   65, 24,
   67, 24
};

/* generic tuner interface functions, shall be moved to the tuner implementations */
int e4000_init(void *dev) {
	rtlsdr_dev_t* devt = (rtlsdr_dev_t*)dev;
	rtlsdr_get_xtal_freq(devt, NULL, &devt->e4k_s.fosc);
	devt->e4k_s.rtl_dev = dev;
	return e4k_init(&devt->e4k_s);
}
int e4000_exit(void *dev) {
	rtlsdr_dev_t* devt = (rtlsdr_dev_t*)dev;
	return e4k_standby(&devt->e4k_s, 1);
}
int e4000_set_freq(void *dev, uint32_t freq) {
	rtlsdr_dev_t* devt = (rtlsdr_dev_t*)dev;
	return e4k_tune_freq(&devt->e4k_s, freq);
}

int e4000_set_bw(void *dev, uint32_t bw) {
	int r = 0;
	rtlsdr_dev_t* devt = (rtlsdr_dev_t*)dev;
	r |= e4k_if_filter_bw_set(&devt->e4k_s, E4K_IF_FILTER_MIX, bw);
	r |= e4k_if_filter_bw_set(&devt->e4k_s, E4K_IF_FILTER_RC, bw);
	return r;
}

int e4000_set_mixer_gain(void *dev, uint32_t gain) {
	rtlsdr_dev_t* devt = (rtlsdr_dev_t*)dev;
	return e4k_set_mixer_gain(&devt->e4k_s, gain);
}

int e4000_set_lna_gain(void *dev, uint32_t gain) {
	rtlsdr_dev_t* devt = (rtlsdr_dev_t*)dev;
	return e4k_set_lna_gain(&devt->e4k_s, gain);
}

int e4000_set_gain_mode(void *dev, uint8_t manual) {
	rtlsdr_dev_t* devt = (rtlsdr_dev_t*)dev;
	return e4k_enable_manual_gain(&devt->e4k_s, manual);
}

int r820t_init(void *dev) {
	rtlsdr_dev_t* devt = (rtlsdr_dev_t*)dev;
	rtlsdr_get_xtal_freq(devt, NULL, &devt->r82xx_p.xtal);
	devt->r82xx_p.rtl_dev = dev;
	return r82xx_init(&devt->r82xx_p);
}

int r820t_exit(void *dev) {
	rtlsdr_dev_t* devt = (rtlsdr_dev_t*)dev;
	return r82xx_standby(&devt->r82xx_p);
}

int r820t_set_freq(void *dev, uint32_t freq) {
    rtlsdr_dev_t* devt = (rtlsdr_dev_t*)dev;
    return r82xx_set_freq(&devt->r82xx_p, freq);
}

int r820t_set_bw(void *dev, uint32_t bw) {
    rtlsdr_dev_t* devt = (rtlsdr_dev_t*)dev;
    return r82xx_set_bw(&devt->r82xx_p, bw);
}

int r820t_set_mixer_gain(void *dev, uint32_t gain) {
	rtlsdr_dev_t* devt = (rtlsdr_dev_t*)dev;
	return r82xx_set_mixer_gain(&devt->r82xx_p, gain);
}

int r820t_set_lna_gain(void *dev, uint32_t gain) {
	rtlsdr_dev_t* devt = (rtlsdr_dev_t*)dev;
	return r82xx_set_lna_gain(&devt->r82xx_p, gain);
}

int r820t_set_gain_mode(void *dev, uint8_t manual) {
	rtlsdr_dev_t* devt = (rtlsdr_dev_t*)dev;
	return r82xx_enable_manual_gain(&devt->r82xx_p, manual);
}

/* definition order must match enum rtlsdr_tuner */
static rtlsdr_tuner_iface_t tuners[] = {
	{
		NULL, NULL, NULL, NULL, NULL, NULL, NULL /* dummy for unknown tuners */
	},
	{
		e4000_init, e4000_exit,
		e4000_set_freq, e4000_set_bw, e4000_set_lna_gain, e4000_set_mixer_gain,
		e4000_set_gain_mode
	},
	{
		r820t_init, r820t_exit,
		r820t_set_freq, r820t_set_bw, r820t_set_lna_gain, r820t_set_mixer_gain,
		r820t_set_gain_mode
	},
};

typedef struct rtlsdr_dongle {
	uint16_t vid;
	uint16_t pid;
	const char *name;
} rtlsdr_dongle_t;

/*
 * Please add your device here and send a patch to osmocom-sdr@lists.osmocom.org
 */
static rtlsdr_dongle_t known_devices[] = {
	{ 0x0bda, 0x2832, "Generic RTL2832U" },
	{ 0x0bda, 0x2838, "Generic RTL2832U OEM" },
	{ 0x0413, 0x6680, "DigitalNow Quad DVB-T PCI-E card" },
	{ 0x0413, 0x6f0f, "Leadtek WinFast DTV Dongle mini D" },
	{ 0x0458, 0x707f, "Genius TVGo DVB-T03 USB dongle (Ver. B)" },
	{ 0x0ccd, 0x00a9, "Terratec Cinergy T Stick Black (rev 1)" },
	{ 0x0ccd, 0x00b3, "Terratec NOXON DAB/DAB+ USB dongle (rev 1)" },
	{ 0x0ccd, 0x00b4, "Terratec Deutschlandradio DAB Stick" },
	{ 0x0ccd, 0x00b5, "Terratec NOXON DAB Stick - Radio Energy" },
	{ 0x0ccd, 0x00b7, "Terratec Media Broadcast DAB Stick" },
	{ 0x0ccd, 0x00b8, "Terratec BR DAB Stick" },
	{ 0x0ccd, 0x00b9, "Terratec WDR DAB Stick" },
	{ 0x0ccd, 0x00c0, "Terratec MuellerVerlag DAB Stick" },
	{ 0x0ccd, 0x00c6, "Terratec Fraunhofer DAB Stick" },
	{ 0x0ccd, 0x00d3, "Terratec Cinergy T Stick RC (Rev.3)" },
	{ 0x0ccd, 0x00d7, "Terratec T Stick PLUS" },
	{ 0x0ccd, 0x00e0, "Terratec NOXON DAB/DAB+ USB dongle (rev 2)" },
	{ 0x1554, 0x5020, "PixelView PV-DT235U(RN)" },
	{ 0x15f4, 0x0131, "Astrometa DVB-T/DVB-T2" },
	{ 0x185b, 0x0620, "Compro Videomate U620F"},
	{ 0x185b, 0x0650, "Compro Videomate U650F"},
	{ 0x185b, 0x0680, "Compro Videomate U680F"},
	{ 0x1b80, 0xd393, "GIGABYTE GT-U7300" },
	{ 0x1b80, 0xd394, "DIKOM USB-DVBT HD" },
	{ 0x1b80, 0xd395, "Peak 102569AGPK" },
	{ 0x1b80, 0xd397, "KWorld KW-UB450-T USB DVB-T Pico TV" },
	{ 0x1b80, 0xd398, "Zaapa ZT-MINDVBZP" },
	{ 0x1b80, 0xd39d, "SVEON STV20 DVB-T USB & FM" },
	{ 0x1b80, 0xd3a4, "Twintech UT-40" },
	{ 0x1b80, 0xd3a8, "ASUS U3100MINI_PLUS_V2" },
	{ 0x1b80, 0xd3af, "SVEON STV27 DVB-T USB & FM" },
	{ 0x1b80, 0xd3b0, "SVEON STV21 DVB-T USB & FM" },
	{ 0x1f4d, 0xa803, "Sweex DVB-T USB" },
	{ 0x1f4d, 0xb803, "GTek T803" },
	{ 0x1f4d, 0xc803, "Lifeview LV5TDeluxe" },
	{ 0x1f4d, 0xd286, "MyGica TD312" },
	{ 0x1f4d, 0xd803, "PROlectrix DV107669" },
#if 0
    { 0x1d50, 0x604b, "HackRF Jawbreaker" },
    { 0x1d50, 0x6089, "HackRF One" },
#endif
};

#define DEFAULT_BUF_NUMBER	15
#define DEFAULT_BUF_LENGTH	(16 * 32 * 512)

#define DEF_RTL_XTAL_FREQ	28800000
#define MIN_RTL_XTAL_FREQ	(DEF_RTL_XTAL_FREQ - 1000)
#define MAX_RTL_XTAL_FREQ	(DEF_RTL_XTAL_FREQ + 1000)

#define CTRL_IN		(0x40 | ENDPOINT_IN)
#define CTRL_OUT	(0x40 | ENDPOINT_OUT)
#define CTRL_TIMEOUT	300

#define EEPROM_ADDR	0xa0

enum usb_reg {
	USB_SYSCTL		= 0x2000,
	USB_CTRL		= 0x2010,
	USB_STAT		= 0x2014,
	USB_EPA_CFG		= 0x2144,
	USB_EPA_CTL		= 0x2148,
	USB_EPA_MAXPKT		= 0x2158,
	USB_EPA_MAXPKT_2	= 0x215a,
	USB_EPA_FIFO_CFG	= 0x2160,
};

enum sys_reg {
	DEMOD_CTL		= 0x3000,
	GPO			= 0x3001,
	GPI			= 0x3002,
	GPOE			= 0x3003,
	GPD			= 0x3004,
	SYSINTE			= 0x3005,
	SYSINTS			= 0x3006,
	GP_CFG0			= 0x3007,
	GP_CFG1			= 0x3008,
	SYSINTE_1		= 0x3009,
	SYSINTS_1		= 0x300a,
	DEMOD_CTL_1		= 0x300b,
	IR_SUSPEND		= 0x300c,
};

enum blocks {
	DEMODB			= 0,
	USBB			= 1,
	SYSB			= 2,
	TUNB			= 3,
	ROMB			= 4,
	IRB			= 5,
	IICB			= 6,
};

/*
 * Copy src to string dst of size siz.  At most siz-1 characters
 * will be copied.  Always NUL terminates (unless siz == 0).
 * Returns strlen(src); if retval >= siz, truncation occurred.
 */
unsigned int rtl_strlcpy(char *dst, const char *src, unsigned int siz)
{
    char *d = dst;
    const char *s = src;
    unsigned int n = siz;

    /* Copy as many bytes as will fit */
    if (n != 0) {
        while (--n != 0) {
            if ((*d++ = *s++) == '\0')
                break;
        }
    }

    /* Not enough room in dst, add NUL and return */
    if (n == 0) {
        if (siz != 0)
            *d = '\0';      /* NUL-terminate dst */
        return siz;
    }

    return(s - src - 1);    /* count does not include NUL */
}

struct usbfs_getdriver {
    unsigned int interface;
    char driver[256];
};

#define IOCTL_USBFS_GETDRIVER   _IOW('U', 8, struct usbfs_getdriver)

int op_kernel_driver_active(int fd, int interface, char *name, unsigned int namelen)
{
	struct usbfs_getdriver getdrv;
	int r;

	getdrv.interface = interface;
	r = ioctl(fd, IOCTL_USBFS_GETDRIVER, &getdrv);
	if (r < 0) {
        if (errno == ENODATA) {
            return 0;
        } else {
            return -errno;
        }
	}

    if (namelen > 255) namelen = 255;
    rtl_strlcpy(name, getdrv.driver, namelen - 1);
    name[namelen] = '\0';
	return 1;
}

int rtlsdr_read_array(rtlsdr_dev_t *dev, uint8_t block, uint16_t addr, uint8_t *array, uint8_t len)
{
	int r;
	uint16_t index = (block << 8);

	r = libusb_control_transfer(dev->devh, CTRL_IN, 0, addr, index, array, len, CTRL_TIMEOUT);
#if 0
	if (r < 0)
		rtlsdr_printf("%s failed with %d\n", __FUNCTION__, r);
#endif
	return r;
}

int rtlsdr_write_array(rtlsdr_dev_t *dev, uint8_t block, uint16_t addr, uint8_t *array, uint8_t len)
{
	int r;
	uint16_t index = (block << 8) | 0x10;

	r = libusb_control_transfer(dev->devh, CTRL_OUT, 0, addr, index, array, len, CTRL_TIMEOUT);
#if 0
	if (r < 0)
		rtlsdr_printf("%s failed with %d\n", __FUNCTION__, r);
#endif
	return r;
}

static inline void rtlsdr_i2c_read_reg(rtlsdr_dev_t *dev, uint16_t addr, uint8_t reg, uint8_t *data)
{
	rtlsdr_write_array(dev, IICB, addr, &reg, 1);
	rtlsdr_read_array(dev, IICB, addr, data, 1);
}

static void rtlsdr_write_to_stderr(void *arg, const char *str, unsigned int len)
{
    write(2, str, len);
}
void (*rtlsdr_display_text)(void *arg, const char *str, unsigned int len) = rtlsdr_write_to_stderr;

void rtlsdr_printf(const char *fmt, ...)
{
	char buf[1024];
    int len;
	va_list	ap;

	va_start(ap, fmt);
	len = vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);
    //buf[len++] = '\n';
    rtlsdr_display_text(NULL, buf, len);
}

static uint16_t rtlsdr_read_reg(rtlsdr_dev_t *dev, uint8_t block, uint16_t addr, uint8_t len)
{
	int r;
	unsigned char data[2];
	uint16_t index = (block << 8);
	uint16_t reg;

	r = libusb_control_transfer(dev->devh, CTRL_IN, 0, addr, index, data, len, CTRL_TIMEOUT);

	if (r < 0)
		rtlsdr_printf("%s failed with %d\n", __FUNCTION__, r);

	reg = (data[1] << 8) | data[0];

	return reg;
}

static int rtlsdr_write_reg(rtlsdr_dev_t *dev, uint8_t block, uint16_t addr, uint16_t val, uint8_t len)
{
	int r;
	unsigned char data[2];

	uint16_t index = (block << 8) | 0x10;

	if (len == 1)
		data[0] = val & 0xff;
	else
		data[0] = val >> 8;

	data[1] = val & 0xff;

	r = libusb_control_transfer(dev->devh, CTRL_OUT, 0, addr, index, data, len, CTRL_TIMEOUT);

	if (r < 0)
		rtlsdr_printf("%s failed with %d\n", __FUNCTION__, r);

	return r;
}

static uint16_t rtlsdr_demod_read_reg(rtlsdr_dev_t *dev, uint8_t page, uint16_t addr, uint8_t len)
{
	int r;
	unsigned char data[2];

	uint16_t index = page;
	uint16_t reg;
	addr = (addr << 8) | 0x20;

	r = libusb_control_transfer(dev->devh, CTRL_IN, 0, addr, index, data, len, CTRL_TIMEOUT);

	if (r < 0)
		rtlsdr_printf("%s failed with %d\n", __FUNCTION__, r);

	reg = (data[1] << 8) | data[0];
	return reg;
}

static int rtlsdr_demod_write_reg(rtlsdr_dev_t *dev, uint8_t page, uint16_t addr, uint16_t val, uint8_t len)
{
	int r;
	unsigned char data[2];
	uint16_t index = 0x10 | page;
	addr = (addr << 8) | 0x20;

	if (len == 1)
		data[0] = val & 0xff;
	else
		data[0] = val >> 8;

	data[1] = val & 0xff;

	r = libusb_control_transfer(dev->devh, CTRL_OUT, 0, addr, index, data, len, CTRL_TIMEOUT);

	if (r < 0)
		rtlsdr_printf("%s failed with %d\n", __FUNCTION__, r);

	rtlsdr_demod_read_reg(dev, 0x0a, 0x01, 1);
	return (r == len) ? 0 : -1;
}

void rtlsdr_set_gpio_bit(rtlsdr_dev_t *dev, uint8_t gpio, int val)
{
	uint16_t r;

	gpio = 1 << gpio;
	r = rtlsdr_read_reg(dev, SYSB, GPO, 1);
	r = val ? (r | gpio) : (r & ~gpio);
	rtlsdr_write_reg(dev, SYSB, GPO, r, 1);
}

void rtlsdr_set_gpio_output(rtlsdr_dev_t *dev, uint8_t gpio)
{
	int r;
	gpio = 1 << gpio;

	r = rtlsdr_read_reg(dev, SYSB, GPD, 1);
	rtlsdr_write_reg(dev, SYSB, GPO, r & ~gpio, 1);
	r = rtlsdr_read_reg(dev, SYSB, GPOE, 1);
	rtlsdr_write_reg(dev, SYSB, GPOE, r | gpio, 1);
}

void rtlsdr_set_i2c_repeater(rtlsdr_dev_t *dev, int on)
{
	rtlsdr_demod_write_reg(dev, 1, 0x01, on ? 0x18 : 0x10, 1);
}

static int rtlsdr_set_fir(rtlsdr_dev_t *dev, int *fir_coeffs)
{
    unsigned int i;
	uint8_t fir[20];

	/* format: int8_t[8] */
	for (i = 0; i < 8; ++i) {
		const int val = fir_coeffs[i];
		if (val < -128 || val > 127) {
			return -1;
		}
		fir[i] = val;
	}
	/* format: int12_t[8] */
	for (i = 0; i < 4; i++) {
		const int val0 = fir_coeffs[8+2*i];
		const int val1 = fir_coeffs[8+2*i+1];
		if (val0 < -2048 || val0 > 2047 || val1 < -2048 || val1 > 2047) {
			return -1;
		}
		fir[8+i*3  ] = val0 >> 4;
		fir[8+i*3+1] = (val0 << 4) | ((val1 >> 8) & 0x0f);
		fir[8+i*3+2] = (val1 & 0xff);
	}

	for (i = 0; i < 20; i++) {
		if (rtlsdr_demod_write_reg(dev, 1, 0x1c + i, fir[i], 1))
			return -1;
	}

	return 0;
}

static void rtlsdr_init_baseband(rtlsdr_dev_t *dev)
{
	unsigned int i;

	/* initialize USB */
	rtlsdr_write_reg(dev, USBB, USB_SYSCTL, 0x09, 1);
	rtlsdr_write_reg(dev, USBB, USB_EPA_MAXPKT, 0x0002, 2);
	rtlsdr_write_reg(dev, USBB, USB_EPA_CTL, 0x1002, 2);

	/* poweron demod */
	rtlsdr_write_reg(dev, SYSB, DEMOD_CTL_1, 0x22, 1);
	rtlsdr_write_reg(dev, SYSB, DEMOD_CTL, 0xe8, 1);

	/* reset demod (bit 3, soft_rst) */
	rtlsdr_demod_write_reg(dev, 1, 0x01, 0x14, 1);
	rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1);

	/* disable spectrum inversion and adjacent channel rejection */
	rtlsdr_demod_write_reg(dev, 1, 0x15, 0x00, 1);
	dev->spectrum_inversion = 0;
	rtlsdr_demod_write_reg(dev, 1, 0x16, 0x0000, 2);

	/* clear both DDC shift and IF frequency registers  */
	for (i = 0; i < 6; i++)
		rtlsdr_demod_write_reg(dev, 1, 0x16 + i, 0x00, 1);

	rtlsdr_set_fir(dev, fir_default);

	/* enable SDR mode, disable DAGC (bit 5) */
	rtlsdr_demod_write_reg(dev, 0, 0x19, 0x05, 1);

	/* init FSM state-holding register */
	rtlsdr_demod_write_reg(dev, 1, 0x93, 0xf0, 1);
	rtlsdr_demod_write_reg(dev, 1, 0x94, 0x0f, 1);

	/* disable AGC (en_dagc, bit 0) (this seems to have no effect) */
	rtlsdr_demod_write_reg(dev, 1, 0x11, 0x00, 1);

	/* disable RF and IF AGC loop */
	rtlsdr_demod_write_reg(dev, 1, 0x04, 0x00, 1);

	/* disable PID filter (enable_PID = 0) */
	rtlsdr_demod_write_reg(dev, 0, 0x61, 0x60, 1);

	/* opt_adc_iq = 0, default ADC_I/ADC_Q datapath */
	rtlsdr_demod_write_reg(dev, 0, 0x06, 0x80, 1);

	/* Enable Zero-IF mode (en_bbin bit), DC cancellation (en_dc_est),
	 * IQ estimation/compensation (en_iq_comp, en_iq_est) */
	rtlsdr_demod_write_reg(dev, 1, 0xb1, 0x1b, 1);

	/* disable 4.096 MHz clock output on pin TP_CK0 */
	rtlsdr_demod_write_reg(dev, 0, 0x0d, 0x83, 1);
}

int rtlsdr_deinit_baseband(rtlsdr_dev_t *dev)
{
	int r = 0;

	if (!dev)
		return -1;

	if (dev->tuner && dev->tuner->exit) {
		rtlsdr_set_i2c_repeater(dev, 1);
		r = dev->tuner->exit(dev); /* deinitialize tuner */
		rtlsdr_set_i2c_repeater(dev, 0);
	}

	/* poweroff demodulator and ADCs */
	rtlsdr_write_reg(dev, SYSB, DEMOD_CTL, 0x20, 1);

	return r;
}

int rtlsdr_set_if_freq(rtlsdr_dev_t *dev, uint32_t freq)
{
    uint32_t rtl_xtal;
    int32_t if_freq;
    uint8_t tmp;
    int r;

    if (!dev)
        return -1;

    /* read corrected clock value */
    rtl_xtal = (uint32_t)APPLY_PPM_CORR(dev->rtl_xtal, dev->corr);
    if_freq = ((freq * TWO_POW(22)) / rtl_xtal) * (-1);

    tmp = (if_freq >> 16) & 0x3f;
    r = rtlsdr_demod_write_reg(dev, 1, 0x19, tmp, 1);
    tmp = (if_freq >> 8) & 0xff;
    r |= rtlsdr_demod_write_reg(dev, 1, 0x1a, tmp, 1);
    tmp = if_freq & 0xff;
    r |= rtlsdr_demod_write_reg(dev, 1, 0x1b, tmp, 1);

    return r;
}

int rtlsdr_set_sample_freq_correction(rtlsdr_dev_t *dev, int ppm)
{
	int r = 0;
	uint8_t tmp;
	int16_t offs = ppm * (-1) * TWO_POW(24) / 1000000;

	tmp = offs & 0xff;
	r |= rtlsdr_demod_write_reg(dev, 1, 0x3f, tmp, 1);
	tmp = (offs >> 8) & 0x3f;
	r |= rtlsdr_demod_write_reg(dev, 1, 0x3e, tmp, 1);
	return r;
}

int rtlsdr_set_xtal_freq(rtlsdr_dev_t *dev, uint32_t rtl_freq, uint32_t tuner_freq)
{
	int r = 0;

	if (!dev)
		return -1;

	if (rtl_freq > 0 &&
		(rtl_freq < MIN_RTL_XTAL_FREQ || rtl_freq > MAX_RTL_XTAL_FREQ))
		return -2;

	if (rtl_freq > 0 && dev->rtl_xtal != rtl_freq) {
		dev->rtl_xtal = rtl_freq;

		/* update xtal-dependent settings */
		if (dev->rate)
			r = rtlsdr_set_sample_rate(dev, dev->rate);
	}

	if (dev->tun_xtal != tuner_freq) {
		if (0 == tuner_freq)
			dev->tun_xtal = dev->rtl_xtal;
		else
			dev->tun_xtal = tuner_freq;

		/* read corrected clock value into e4k and r82xx structure */
		if (rtlsdr_get_xtal_freq(dev, NULL, &dev->e4k_s.fosc) ||
		    rtlsdr_get_xtal_freq(dev, NULL, &dev->r82xx_p.xtal))
			return -3;

		/* update xtal-dependent settings */
		if (dev->freq)
			r = rtlsdr_set_center_freq(dev, dev->freq);
	}

	return r;
}

int rtlsdr_get_xtal_freq(rtlsdr_dev_t *dev, uint32_t *rtl_freq, uint32_t *tuner_freq)
{
	if (!dev)
		return -1;

	if (rtl_freq)
		*rtl_freq = (uint32_t) APPLY_PPM_CORR(dev->rtl_xtal, dev->corr);

	if (tuner_freq)
		*tuner_freq = (uint32_t) APPLY_PPM_CORR(dev->tun_xtal, dev->corr);

	return 0;
}

static inline unsigned int rtl_ucs2_to_utf8(char *utf8, uint16_t ucs2)
{
  if (ucs2 <= 0x7f) {
    utf8[0] = ucs2;
    return 1;
  }
  else if (ucs2 <= 0x7ff) {
    utf8[0] = 0xc0 | ((ucs2 >>  6) & 0x1f);
    utf8[1] = 0x80 | ((ucs2 >>  0) & 0x3f);
    return 2;
  }
  else {
    utf8[0] = 0xe0 | ((ucs2 >> 12) & 0x0f);
    utf8[1] = 0x80 | ((ucs2 >>  6) & 0x3f);
    utf8[2] = 0x80 | ((ucs2 >>  0) & 0x3f);
    return 3;
  }
}

static int rtlsdr_get_string_descriptor_utf8(libusb_device_handle *dev, uint8_t desc_index,
                                             unsigned char *data, unsigned int length)
{
    unsigned char tbuf[257]; /* Some devices choke on size > 255 */
    uint16_t *ucs2buf = (uint16_t*)tbuf;
    unsigned int di, si;
    int r;
    uint16_t langid;

    /* Asking for the zero'th index is special - it returns a string
     * descriptor that contains all the language IDs supported by the
     * device. Typically there aren't many - often only one. Language
     * IDs are 16 bit numbers, and they start at the third byte in the
     * descriptor. There's also no point in trying to read descriptor 0
     * with this function. See USB 2.0 specification section 9.6.7 for
     * more information.
     */

    if (desc_index == 0)
        return LIBUSB_ERROR_INVALID_PARAM;

    r = libusb_get_string_descriptor(dev, 0, 0, tbuf, 255);
    if (r < 0)
        return r;

    if (r < 4)
        return LIBUSB_ERROR_IO;

    langid = tbuf[2] | (tbuf[3] << 8);
    r = libusb_get_string_descriptor(dev, desc_index, langid, tbuf, 255);
    if (r < 0)
        return r;

    if ((tbuf[1] != LIBUSB_DT_STRING) || (tbuf[0] > r))
        return LIBUSB_ERROR_IO;

    for (di = 0, si = 1; 2*si < tbuf[0]; si++) {
        if (di >= (length - 1))
            break;
        di += rtl_ucs2_to_utf8(data+di, ucs2buf[si]);
    }

    data[di] = 0;
    return di;
}

int rtlsdr_get_usb_strings(rtlsdr_dev_t *dev, char *serial)
{
    struct libusb_device_descriptor dd;
    libusb_device *device = NULL;
    int r = 0;

    if (!dev->devh)
        return -1;

    device = libusb_get_device(dev->devh);
    r = libusb_get_device_descriptor(device, &dd);
    if (r < 0)
        return -1;

    if (serial) {
        memset(serial, 0, 256);
        rtlsdr_get_string_descriptor_utf8(dev->devh, dd.iSerialNumber, (unsigned char *)serial, 255);
    }

    return 0;
}

int rtlsdr_write_eeprom(rtlsdr_dev_t *dev, uint8_t *data, uint8_t offset, uint16_t len)
{
	int r = 0;
	int i;
	uint8_t cmd[2];

	if (!dev)
		return -1;

	if ((len + offset) > 256)
		return -2;

	for (i = 0; i < len; i++) {
		cmd[0] = i + offset;
		r = rtlsdr_write_array(dev, IICB, EEPROM_ADDR, cmd, 1);
		r = rtlsdr_read_array(dev, IICB, EEPROM_ADDR, &cmd[1], 1);

		/* only write the byte if it differs */
		if (cmd[1] == data[i])
			continue;

		cmd[1] = data[i];
		r = rtlsdr_write_array(dev, IICB, EEPROM_ADDR, cmd, 2);
		if (r != sizeof(cmd))
			return -3;

		/* for some EEPROMs (e.g. ATC 240LC02) we need a delay
		 * between write operations, otherwise they will fail */
#ifdef _WIN32
		Sleep(5);
#else
		usleep(5000);
#endif
	}

	return 0;
}

int rtlsdr_read_eeprom(rtlsdr_dev_t *dev, uint8_t *data, uint8_t offset, uint16_t len)
{
	int r = 0;
	int i;

	if (!dev)
		return -1;

	if ((len + offset) > 256)
		return -2;

	r = rtlsdr_write_array(dev, IICB, EEPROM_ADDR, &offset, 1);
	if (r < 0)
		return -3;

	for (i = 0; i < len; i++) {
		r = rtlsdr_read_array(dev, IICB, EEPROM_ADDR, data + i, 1);

		if (r < 0)
			return -3;
	}

	return r;
}

static int set_spectrum_inversion(rtlsdr_dev_t *dev, int inverted)
{
	int r = 0;

	if (dev->spectrum_inversion == inverted)
		return r;

	r |= rtlsdr_demod_write_reg(dev, 1, 0x15, inverted, 1);
	dev->spectrum_inversion = inverted;
	return r;
}

int rtlsdr_set_center_freq(rtlsdr_dev_t *dev, uint32_t freq)
{
    int r = -1;

    if (!dev || !dev->tuner)
        return -1;

    if (dev->tuner_type == RTLSDR_TUNER_UNKNOWN) {
        r = rtlsdr_set_if_freq(dev, freq);
    } else if (dev->tuner && dev->tuner->set_freq) {
        rtlsdr_set_i2c_repeater(dev, 1);
        r = dev->tuner->set_freq(dev, freq - dev->offs_freq);
        rtlsdr_set_i2c_repeater(dev, 0);
    }

    if (!r)
        dev->freq = freq;
    else
        dev->freq = 0;

    return r;
}

uint32_t rtlsdr_get_center_freq(rtlsdr_dev_t *dev)
{
    if (!dev)
        return 0;

    return dev->freq;
}

int rtlsdr_set_tuner_lna_gain(rtlsdr_dev_t *dev, unsigned int gain)
{
	int r = 0;
    unsigned int lna_gain = 0;

    if (gain > 15) gain = 15;

	if (dev->tuner_type == RTLSDR_TUNER_E4000) {
        lna_gain = e4k_lna_mixer_gains[2*gain];
    } else {
        lna_gain = r82xx_lna_mixer_gains[2*gain];
    }

	if (dev->tuner->set_lna_gain) {
		rtlsdr_set_i2c_repeater(dev, 1);
		r = dev->tuner->set_lna_gain((void *)dev, lna_gain);
		rtlsdr_set_i2c_repeater(dev, 0);
	}
	return lna_gain;
}

int rtlsdr_set_tuner_mixer_gain(rtlsdr_dev_t *dev, unsigned int gain)
{
	int r = 0;
    unsigned int mixer_gain = 0;

    if (gain > 15) gain = 15;

	if (dev->tuner_type == RTLSDR_TUNER_E4000) {
        mixer_gain = e4k_lna_mixer_gains[2*gain+1];
    } else {
        mixer_gain = r82xx_lna_mixer_gains[2*gain+1];
    }

	if (dev->tuner->set_lna_gain) {
		rtlsdr_set_i2c_repeater(dev, 1);
		r = dev->tuner->set_mixer_gain((void *)dev, mixer_gain);
		rtlsdr_set_i2c_repeater(dev, 0);
	}
	return mixer_gain;
}

int rtlsdr_set_freq_correction(rtlsdr_dev_t *dev, int ppm)
{
	int r = 0;

	if (!dev)
		return -1;

	if (dev->corr == ppm)
		return -2;

	dev->corr = ppm;

	r |= rtlsdr_set_sample_freq_correction(dev, ppm);

	/* read corrected clock value into e4k and r82xx structure */
	if (rtlsdr_get_xtal_freq(dev, NULL, &dev->e4k_s.fosc) ||
	    rtlsdr_get_xtal_freq(dev, NULL, &dev->r82xx_p.xtal))
		return -3;

	if (dev->freq) /* retune to apply new correction value */
		r |= rtlsdr_set_center_freq(dev, dev->freq);

	return r;
}

int rtlsdr_get_freq_correction(rtlsdr_dev_t *dev)
{
	if (!dev)
		return 0;

	return dev->corr;
}

enum rtlsdr_tuner rtlsdr_get_tuner_type(rtlsdr_dev_t *dev)
{
	if (!dev)
		return RTLSDR_TUNER_UNKNOWN;

	return dev->tuner_type;
}

int rtlsdr_set_tuner_gain(rtlsdr_dev_t *dev, unsigned int gain)
{
	int r = 0;
    unsigned int i, lna_gain = 0, mixer_gain = 0;

	if (!dev || !dev->tuner)
		return -1;

    for (i = 0; i < 19; i++) {
        if (25*i >= gain)
            break;
    }

	if (dev->tuner_type == RTLSDR_TUNER_E4000) {
        lna_gain = e4k_lna_mixer_gains[2*i];
        mixer_gain = e4k_lna_mixer_gains[2*i+1];
    } else {
        lna_gain = r82xx_lna_mixer_gains[2*i];
        mixer_gain = r82xx_lna_mixer_gains[2*i+1];
    }

	if (dev->tuner->set_lna_gain) {
		rtlsdr_set_i2c_repeater(dev, 1);
		r = dev->tuner->set_lna_gain((void *)dev, lna_gain);
		r = dev->tuner->set_mixer_gain((void *)dev, mixer_gain);
		rtlsdr_set_i2c_repeater(dev, 0);
	}

	if (!r)
		dev->gain = gain;
	else
		dev->gain = 0;

	return r;
}

int rtlsdr_get_tuner_gain(rtlsdr_dev_t *dev)
{
	if (!dev)
		return 0;

	return dev->gain;
}

int rtlsdr_set_tuner_gain_mode(rtlsdr_dev_t *dev, int mode)
{
	int r = 0;

	if (!dev || !dev->tuner)
		return -1;

	if (dev->tuner->set_gain_mode) {
		rtlsdr_set_i2c_repeater(dev, 1);
		r = dev->tuner->set_gain_mode((void *)dev, mode);
		rtlsdr_set_i2c_repeater(dev, 0);
	}

	return r;
}

int rtlsdr_set_sample_rate(rtlsdr_dev_t *dev, uint32_t samp_rate)
{
	int r = 0;
	uint16_t tmp;
	uint32_t rsamp_ratio, real_rsamp_ratio;
	float real_rate;

	if (!dev)
		return -1;

	/* check if the rate is supported by the resampler */
	if ((samp_rate <= 225000) || (samp_rate > 3200000) ||
	   ((samp_rate > 300000) && (samp_rate <= 900000))) {
		rtlsdr_printf("Invalid sample rate: %u Hz\n", samp_rate);
		return -EINVAL;
	}

	rsamp_ratio = (dev->rtl_xtal * TWO_POW(22)) / samp_rate;
	rsamp_ratio &= 0x0ffffffc;

	real_rsamp_ratio = rsamp_ratio | ((rsamp_ratio & 0x08000000) << 1);
	real_rate = (dev->rtl_xtal * TWO_POW(22)) / real_rsamp_ratio;

	if (((float)samp_rate) != real_rate)
		rtlsdr_printf("Exact sample rate is: %f Hz\n", real_rate);

	if (dev->tuner && dev->tuner->set_bw) {
		rtlsdr_set_i2c_repeater(dev, 1);
		dev->tuner->set_bw(dev, (int)real_rate);
		rtlsdr_set_i2c_repeater(dev, 0);
	}

	dev->rate = (uint32_t)real_rate;

	tmp = (rsamp_ratio >> 16);
	r |= rtlsdr_demod_write_reg(dev, 1, 0x9f, tmp, 2);
	tmp = rsamp_ratio & 0xffff;
	r |= rtlsdr_demod_write_reg(dev, 1, 0xa1, tmp, 2);

	r |= rtlsdr_set_sample_freq_correction(dev, dev->corr);

	/* reset demod (bit 3, soft_rst) */
	r |= rtlsdr_demod_write_reg(dev, 1, 0x01, 0x14, 1);
	r |= rtlsdr_demod_write_reg(dev, 1, 0x01, 0x10, 1);

    /* recalculate offset frequency if offset tuning is enabled */
    if (dev->offs_freq)
        rtlsdr_set_offset_tuning(dev, 1);

	return r;
}

uint32_t rtlsdr_get_sample_rate(rtlsdr_dev_t *dev)
{
	if (!dev)
		return 0;

	return dev->rate;
}

int rtlsdr_set_testmode(rtlsdr_dev_t *dev, int on)
{
	if (!dev)
		return -1;

	return rtlsdr_demod_write_reg(dev, 0, 0x19, on ? 0x03 : 0x05, 1);
}

int rtlsdr_set_agc_mode(rtlsdr_dev_t *dev, int on)
{
	if (!dev)
		return -1;

	return rtlsdr_demod_write_reg(dev, 0, 0x19, on ? 0x25 : 0x05, 1);
}

int rtlsdr_set_offset_tuning(rtlsdr_dev_t *dev, int on)
{
    unsigned int bw = 0;
    int r = 0;

    if (!dev)
        return -1;

    if (dev->tuner_type != RTLSDR_TUNER_E4000)
        return -2;

    /* based on keenerds 1/f noise measurements */
    dev->offs_freq = on ? ((dev->rate / 2) * 170 / 100) : 0;
    bw = (on ? (2 * dev->offs_freq) : dev->rate);
    r |= rtlsdr_set_if_freq(dev, dev->offs_freq);

    rtlsdr_set_i2c_repeater(dev, 1);
	e4k_if_filter_bw_set(&dev->e4k_s, E4K_IF_FILTER_MIX, bw);
    e4k_if_filter_bw_set(&dev->e4k_s, E4K_IF_FILTER_RC, bw);
    rtlsdr_set_i2c_repeater(dev, 0);

    if (dev->freq > dev->offs_freq)
        r |= rtlsdr_set_center_freq(dev, dev->freq);

    return r;
}

int rtlsdr_get_offset_tuning(rtlsdr_dev_t *dev)
{
    if (!dev)
        return -1;

    return (dev->offs_freq) ? 1 : 0;
}

int rtlsdr_set_dithering(rtlsdr_dev_t *dev, int dither)
{
    if (dev->tuner_type == RTLSDR_TUNER_R820T) {
          return r82xx_set_dither(&dev->r82xx_p, dither);
    }
    return 1;
}

static rtlsdr_dongle_t *find_known_device(uint16_t vid, uint16_t pid)
{
	unsigned int i;
	rtlsdr_dongle_t *device = NULL;

	for (i = 0; i < sizeof(known_devices)/sizeof(rtlsdr_dongle_t); i++ ) {
		if (known_devices[i].vid == vid && known_devices[i].pid == pid) {
			device = &known_devices[i];
			break;
		}
	}

	return device;
}

uint32_t rtlsdr_get_device_count(void)
{
	int i;
	libusb_context *ctx;
	libusb_device **list;
	uint32_t device_count = 0;
	struct libusb_device_descriptor dd;
	ssize_t cnt;

	libusb_init(&ctx);
	cnt = libusb_get_device_list(ctx, &list);
	for (i = 0; i < cnt; i++) {
		libusb_get_device_descriptor(list[i], &dd);
		if (find_known_device(dd.idVendor, dd.idProduct))
			device_count++;
	}

	libusb_free_device_list(list, 1);
	libusb_exit(ctx);
	return device_count;
}

int rtlsdr_get_device_usb_strings(uint32_t index, const char **product, char *serial)
{
	int r = -2;
	libusb_context *ctx;
	libusb_device **list;
	struct libusb_device_descriptor dd;
	rtlsdr_dongle_t *device = NULL;
	rtlsdr_dev_t devt;
	uint32_t i, device_count = 0;
	ssize_t cnt;

	libusb_init(&ctx);

	cnt = libusb_get_device_list(ctx, &list);

	for (i = 0; i < cnt; i++) {
		libusb_get_device_descriptor(list[i], &dd);
		device = find_known_device(dd.idVendor, dd.idProduct);
		if (device) {
			device_count++;
            *product = device->name;
			if (index == device_count - 1) {
				r = libusb_open(list[i], &devt.devh);
				if (!r) {
	                libusb_device *d = libusb_get_device(devt.devh);
	                r = libusb_get_device_descriptor(d, &dd);
	                if (r < 0)
		                return -1;
                    serial[0] = '\0';
	                r = libusb_get_string_descriptor_ascii(devt.devh, dd.iSerialNumber, (unsigned char *)serial, 256);
					libusb_close(devt.devh);
				}
				break;
			}
		}
	}

	libusb_free_device_list(list, 1);

	libusb_exit(ctx);

	return r;
}

int rtlsdr_search_for_device(char *s)
{
    int i, device_count, device;
    char *s2, *product = NULL;
    char serial[256];

    device_count = rtlsdr_get_device_count();
    if (!device_count) {
        rtlsdr_printf("No supported devices found.\n");
        return -1;
    }

    rtlsdr_printf("Found %d device(s):\n", device_count);
    for (i = 0; i < device_count; i++) {
        rtlsdr_get_device_usb_strings(i, (const char**)&product, serial);
        rtlsdr_printf("  %d:  %s, SN: %s\n", i, product, serial);
    }
    /* does string look like raw id number */
    device = (unsigned int)strtoul(s, &s2, 0);
    if (s2[0] == '\0' && device >= 0 && device < device_count) {
        goto device_found;
    }

    /* does string contain a serial */
    for (i = 0; i < device_count; i++) {
        rtlsdr_get_device_usb_strings(i, (const char**)&product, serial);
        if (strstr(s, serial) != 0) {
            continue;
        }
        device = i;
        goto device_found;
    }
    rtlsdr_printf("\nNo matching devices found.\n");
    return -1;

device_found:
    rtlsdr_printf("\nUsing device %d\n", device);
    return device;
}

int rtlsdr_open(rtlsdr_dev_t **out_dev, uint32_t index)
{
	int r;
	int i;
	libusb_device **list;
	rtlsdr_dev_t *dev = NULL;
	libusb_device *device = NULL;
	uint32_t device_count = 0;
	struct libusb_device_descriptor dd;
	uint8_t reg;
	ssize_t cnt;

	dev = malloc(sizeof(rtlsdr_dev_t));
	if (NULL == dev)
		return -ENOMEM;

	memset(dev, 0, sizeof(rtlsdr_dev_t));

	libusb_init(&dev->ctx);

	dev->dev_lost = 1;

	cnt = libusb_get_device_list(dev->ctx, &list);

	for (i = 0; i < cnt; i++) {
		device = list[i];

		libusb_get_device_descriptor(list[i], &dd);

		if (find_known_device(dd.idVendor, dd.idProduct)) {
			device_count++;
		}

		if (index == device_count - 1)
			break;

		device = NULL;
	}

	if (!device) {
		r = -1;
		goto err;
	}

	r = libusb_open(device, &dev->devh);
	if (r < 0) {
		libusb_free_device_list(list, 1);
		rtlsdr_printf("usb_open error %d\n", r);
		if(r == LIBUSB_ERROR_ACCESS)
			rtlsdr_printf("Please fix the device permissions, e.g. by installing the udev rules file rtl-sdr.rules\n");
		goto err;
	}

	libusb_free_device_list(list, 1);

#if 0
	if (op_kernel_driver_active(dev->devh, 0, drivername, 256) == 1) {
        if (!strstr(drivername, "dvb")) {
		    if (!libusb_detach_kernel_driver(dev->devh, 0)) {
			    rtlsdr_printf("Detached kernel driver\n");
		    } else {
			    rtlsdr_printf("Detaching kernel driver failed!\n");
		    }
        } else {
		    rtlsdr_printf("Kernel driver is active, or device is claimed by second instance of librtlsdr.\n"
				          "In the first case, please either detach, blacklist or delte the kernel module (dvb_usb_rtl28xxu),\n"
                          "or enable automatic detaching at compile time.\n\n");
        }
	}
#endif

	if (libusb_kernel_driver_active(dev->devh, 0) == 1) {
		rtlsdr_printf("Kernel driver is active, or device is claimed by second instance of librtlsdr.\n"
				      "In the first case, please either detach, blacklist or delte the kernel module (dvb_usb_rtl28xxu),\n"
                      "or enable automatic detaching at compile time.\n\n");
	}

	r = libusb_claim_interface(dev->devh, 0);
	if (r < 0) {
		rtlsdr_printf("usb_claim_interface error %d\n", r);
		goto err;
	}

	dev->rtl_xtal = DEF_RTL_XTAL_FREQ;

	/* perform a dummy write, if it fails, reset the device */
	if (rtlsdr_write_reg(dev, USBB, USB_SYSCTL, 0x09, 1) < 0) {
		rtlsdr_printf("Resetting device...\n");
		libusb_reset_device(dev->devh);
	}

	rtlsdr_init_baseband(dev);
	dev->dev_lost = 0;

    /*
     * The fc0012 has it's RESET line hooked up to GPIO5
     *
     * If we don't set GPIO5 to an output and leave it floating,
     * the tuner never comes up (just stays in RESET state)
     *
     * After assigning GPIO5 to an output, do a reset of the pin first,
     * before probing for the tuner, to allow the chip to initialize and settle.
     */
	rtlsdr_set_gpio_output(dev, 5);
	rtlsdr_set_gpio_bit(dev, 5, 1);
	rtlsdr_set_gpio_bit(dev, 5, 0);

	/* Probe tuners */
	rtlsdr_set_i2c_repeater(dev, 1);

    reg = 0;
	rtlsdr_i2c_read_reg(dev, E4K_I2C_ADDR, E4K_CHECK_ADDR, &reg);
	if (reg == E4K_CHECK_VAL) {
		rtlsdr_printf("Found Elonics E4000 tuner\n");
		dev->tuner_type = RTLSDR_TUNER_E4000;
		goto found;
	}

    reg = 0;
	rtlsdr_i2c_read_reg(dev, R820T_I2C_ADDR, R82XX_CHECK_ADDR, &reg);
	if (reg == R82XX_CHECK_VAL) {
		rtlsdr_printf("Found Rafael Micro R820T tuner\n");
		dev->r82xx_p.rafael_chip = CHIP_R820T;
		dev->tuner_type = RTLSDR_TUNER_R820T;
		goto found;
	}

    reg = 0;
	rtlsdr_i2c_read_reg(dev, R828D_I2C_ADDR, R82XX_CHECK_ADDR, &reg);
	if (reg == R82XX_CHECK_VAL) {
		rtlsdr_printf("Found Rafael Micro R828D tuner\n");
		dev->r82xx_p.rafael_chip = CHIP_R828D;
		dev->tun_xtal = R828D_XTAL_FREQ;
		dev->tuner_type = RTLSDR_TUNER_R820T;
		goto found;
	}

found:
	/* use the rtl clock value by default */
	dev->tun_xtal = dev->rtl_xtal;
	dev->tuner = &tuners[dev->tuner_type];

	switch (dev->tuner_type) {
	case RTLSDR_TUNER_R820T:
		/* disable Zero-IF mode */
		rtlsdr_demod_write_reg(dev, 1, 0xb1, 0x1a, 1);

		/* only enable In-phase ADC input */
		rtlsdr_demod_write_reg(dev, 0, 0x08, 0x4d, 1);

        /* the R82XX use 3.57 MHz IF for the DVB-T 6 MHz mode, and
         * 4.57 MHz for the 8 MHz mode */
        rtlsdr_set_if_freq(dev, R82XX_IF_FREQ);

        /* enable spectrum inversion?? */
        set_spectrum_inversion(dev, 1);

		break;
	case RTLSDR_TUNER_UNKNOWN:
		rtlsdr_printf("No supported tuner found: attempting to enable direct sampling mode, input 1.\n");

		/* disable Zero-IF mode */
		r |= rtlsdr_demod_write_reg(dev, 1, 0xb1, 0x1a, 1);

		/* only enable In-phase ADC input */
		r |= rtlsdr_demod_write_reg(dev, 0, 0x08, 0x4d, 1);

		/* disable spectrum inversion - no need, hasn't been enabled yet. */
		break;
	default:
		break;
	}

	if (dev->tuner->init) {
		r = dev->tuner->init(dev);
	}

	rtlsdr_set_i2c_repeater(dev, 0);

	*out_dev = dev;

	return 0;
err:
	if (dev) {
		free(dev);
	}

	return r;
}

int rtlsdr_close(rtlsdr_dev_t *dev)
{
	if (!dev)
		return -1;

	if(!dev->dev_lost) {
		rtlsdr_deinit_baseband(dev);
	}

	libusb_release_interface(dev->devh, 0);

	libusb_close(dev->devh);

	libusb_exit(dev->ctx);

	free(dev);

	return 0;
}

int rtlsdr_reset_buffer(rtlsdr_dev_t *dev)
{
	if (!dev)
		return -1;

	rtlsdr_write_reg(dev, USBB, USB_EPA_CTL, 0x1002, 2);
	rtlsdr_write_reg(dev, USBB, USB_EPA_CTL, 0x0000, 2);

	return 0;
}

uint32_t rtlsdr_get_tuner_clock(void *dev)
{
	uint32_t tuner_freq;

	if (!dev)
		return 0;

	/* read corrected clock value */
	if (rtlsdr_get_xtal_freq((rtlsdr_dev_t *)dev, NULL, &tuner_freq))
		return 0;

	return tuner_freq;
}

int rtlsdr_i2c_write_fn(void *dev, uint8_t i2c_addr, uint8_t *buf, int len)
{
	uint16_t addr = i2c_addr;
    if (dev)
        return rtlsdr_write_array(((rtlsdr_dev_t *)dev), IICB, addr, buf, len);

    return -1;
}

int rtlsdr_i2c_read_fn(void *dev, uint8_t i2c_addr, uint8_t *buf, int len)
{
	uint16_t addr = i2c_addr;
    if (dev)
        return rtlsdr_read_array(((rtlsdr_dev_t *)dev), IICB, addr, buf, len);

    return -1;
}

int rtlsdr_read_sync(rtlsdr_dev_t *dev, void *buf, unsigned int len, unsigned int *n_read)
{
	if (!dev)
		return -1;

	return libusb_bulk_transfer(dev->devh, 0x81, buf, len, (int*)n_read, 0);
}

