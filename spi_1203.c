 /*
 * Maxim MAX1202 MAX1203 MAX1204 SPI testing utility (using spidev driver)
 *
 * Copyright (c) 2012 Alberto Panu
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 *
 */

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <time.h>

#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define MAX1202_3_lsb 0.001
#define MAX124_lsb 0.004

static const char *device = "/dev/spidev0.0";
static uint8_t mode = SPI_MODE_0;
static uint8_t bits = 8;
static uint32_t speed = 500000;
static uint16_t delay = 0;
static uint8_t deselect = 0;
static uint8_t input = 0;
static uint8_t verbose = 0;
static uint8_t unipolar = 8;
static uint8_t single = 4;
static uint8_t maxclock = 3;
static uint8_t newline = 1;
static uint8_t raw = 0;
static uint16_t bipolarconvert = 2047;
static float lsb = MAX1202_3_lsb;
static float fullscale = 0;
static uint8_t chip = 0 ; // chip type:
//			     0 for MAX1202 and MAX 1203
//			     1 for MAX1204
static uint8_t inputtable[] = { 0, 64, 16, 80, 32, 96, 48, 112 };

static int sequentialinputs[] = { 1, 3, 5, 7};

static int clockspersecond=1000000;
static int samplingrate=1000;
static int clockspersample;

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static void vexit(const char *s)
{
	printf(s);
	if ( newline || verbose ) {
		printf("\n");
	}
	exit(1);
}

static void print_usage(const char *prog)
{
		printf("Usage: %s [-4Dscibkvnd]\n", prog);
		puts(
		"\n"
		" Maxim MAX1202 MAX1203 MAX1204 SPI adc utility\n"
		" Copyright (C) 2012  Alberto Panu\n"
		" http://www.panu.it/raspandmax/\n"
		"\n"
		" This program comes with ABSOLUTELY NO WARRANTY.\n"
		" This is free software, and you are welcome to redistribute it\n"
		" under certain conditions see GNU GPL v3\n"
		"\n"
		" Usage:\n"
		" -4 --max1204 select max1204 10 bit adc, default is max1202/3 12 bit\n"
		" -f --full full scale value, default are:\n"
		"      0 to 4.095 volt for unipolar mode\n"
		"      -2.048 to 2.047 volt for bipolar mode\n"
		" -D --device device to use (default /dev/spidev0.0)\n"
		" -s --speed SPI bus speed (Hz), default 500000\n"
		" -p --Sampling Rate (Hz), default 1000\n"
		" -c --chipsel disable chipsel at read end, default don't disable\n"
		" -i --input chose input channel 0 to 7, default 0\n"
		" -b --bipolar set to bipolar mode, default unipolar, you need a -5V\n"
		"      power supply on pin 9!\n"
		" -k --clock set internal clock mode, default external\n"
		" -v --verbose print extra info usefoul for debug\n"
		" -r --raw raw mode ouput\n"
		" -n --newline suppress new line at non verbose output end\n"
		" -d --diff set to differential mode,\n"
		"      default is single ended, see table\n"
		"      for input channel selection:\n"
		"\n"
		"       ---------------\n"
		"      | Input l + | - |\n"
		"       ---------------\n"
		"      |   0   | 0 | 1 |\n"
		"       ---------------\n"
		"      |   1   | 2 | 3 |\n"
		"       ---------------\n"
		"      |   2   | 4 | 5 |\n"
		"       ---------------\n"
		"      |   3   | 6 | 7 |\n"
		"       ---------------\n"
		"      |   4   | 1 | 0 |\n"
		"       ---------------\n"
		"      |   5   | 3 | 2 |\n"
		"       ---------------\n"
		"      |   6   | 5 | 4 |\n"
		"       ---------------\n"
		"      |   7   | 7 | 6 |\n"
		"       ---------------\n"
	);
	exit(1);
}

static void parse_opts(int argc, char *argv[])
{
	while (1) {
		static const struct option lopts[] = {
			{ "input", 1, 0,'i' },
			{ "full", 1, 0, 'f' },
			{ "help", 0, 0, 'h' },
			{ "speed", 1, 0, 's' },
			{ "sampling", 1, 0, 'm' },
			{ "device", 1, 0, 'D' },
			{ "verbose", 0, 0, 'v' },
			{ "raw", 0, 0, 'r' },
			{ "max1204", 0, 0, '4' },
			{ "bipolar", 0, 0, 'b' },
			{ "diff", 0, 0, 'd' },
			{ "maxclock", 0, 0, 'k' },
			{ "newline", 0, 0, 'n' },
			{ NULL, 0, 0, 0 },
		};
		int c;

		c = getopt_long(argc, argv, "i:s:f:D:m:v4hbdknr", lopts, NULL);

		if (c == -1)
		break;

		switch (c) {

			case 'D':
			device = optarg;
			break;

			case 'i':
			input = atoi(optarg);
			break;

			case 'f':
			fullscale = atof(optarg);
			break;

			case 's':
			speed = atoi(optarg);
			break;

			case 'm':
			samplingrate = atoi(optarg);
			break;

			case 'v':
			verbose = 1;
			break;

			case 'r':
			raw = 1;
			break;

			case '4':
			chip = 1;
			break;

			case 'b':
			unipolar = 0;
			break;

			case 'd':
			single = 0;
			break;

			case 'k':
			maxclock = 2;
			break;

			case 'n':
			newline=0;
			break;

			default:
			print_usage(argv[0]);
			break;
		}
	}
}

int spi_init()
{
	int fd;
	fd = open(device, O_RDWR);
		if (fd < 0) {
			if ( verbose ) {
				printf("%s\n", device);
				pabort("can't open device");
			} else {
				vexit("E");
			}
		}
	int ret;
	clockspersample=clockspersecond/samplingrate;

	//set SPI Mode
	ret = ioctl(fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1 ) {
		close(fd);
		if ( verbose ) {
			pabort("can't set spi mode");
		} else {
			vexit("E");
		}
	}

	ret = ioctl(fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1) {
		close(fd);
		if ( verbose ) {
			pabort("can't get spi mode");
		} else {
			vexit("E");
		}
	}

	//bits per word
	ret = ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1) {
		close(fd);
		if ( verbose ) {
			pabort("can't set bits per word");
		} else {
			vexit("E");
		}
	}

	ret = ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1) {
		close(fd);
		if ( verbose ) {
			pabort("can't get bits per word");
		} else {
			vexit("E");
		}
	}

	//Max SPI speed (Hz)
	ret = ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
		close(fd);
		if ( verbose ) {
			pabort("can't set max speed hz");
		} else {
			vexit("E");
		}
	}
	ret = ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
	if (ret == -1) {
		close(fd);
		if ( verbose ) {
			pabort("can't get max speed hz");
		} else {
			vexit("E");
		}
	}
	return fd;
}

int get_value(int transmitbyte, int fd){
	int ret;
	uint8_t tx[] = {
		transmitbyte, 0, 0
	};

	uint8_t rx[ARRAY_SIZE(tx)] = {0,};

	struct spi_ioc_transfer tr = {
		.tx_buf = (unsigned long)tx,
		.rx_buf = (unsigned long)rx,
		.len = ARRAY_SIZE(tx),
		.delay_usecs = delay,
		.speed_hz = speed,
		.bits_per_word = bits,
		.cs_change = deselect,
	};

	ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
	if (ret < 1) {
		close(fd);
		if ( verbose ) {
			pabort("can't send spi message");
		} else {
			vexit("E");
		}
	}

	unsigned int first_byte = rx[ARRAY_SIZE(rx) - 3];
	unsigned int second_byte = rx[ARRAY_SIZE(rx) - 2];
	unsigned int third_byte = rx[ARRAY_SIZE(rx) - 1];

	if ( first_byte && !chip ) {
		close(fd);
		if ( verbose ) {
			perror("Protocol error: the first byte is not 0!");
		} else {
			vexit("E");
		}
	}

	if ( second_byte > 127 ) {
		close(fd);
		if ( verbose ) {
			perror("Protocol error: the first bit of the second byte must be 0!");
		} else {
			vexit("E");
		}
	}

	unsigned int padding;
	padding = 7;

	if ( ( third_byte & padding ) != 0 ) {
		close(fd);
		vexit("E");
	}

	signed long receivedword;
	receivedword = third_byte >> 3;
	receivedword |= second_byte << 5;
	//printf("%d\n", receivedword);

	return receivedword;
}



int main(int argc, char *argv[])
{
	int fd;
	int ret = 0;
  int out[ARRAY_SIZE(sequentialinputs)];

	//get console inputs
	parse_opts(argc, argv);

	//open connection to SPI Device
	fd=spi_init();

	uint8_t transmitbyte = 0b10000000;
	transmitbyte |= unipolar;
	transmitbyte |= single;
	transmitbyte |= maxclock;

	uint8_t temptransmitbyte;
	clock_t start_t, end_t;
	clock_t samplestart, sampleend, sampledur[ARRAY_SIZE(sequentialinputs)];

  while(1){
		start_t = clock();
		int i;
		samplestart=clock();
		for (i=0;i<ARRAY_SIZE(sequentialinputs);i++){
			temptransmitbyte=transmitbyte;
			temptransmitbyte |= inputtable[sequentialinputs[i]];
			out[i]=get_value(temptransmitbyte,fd);

			sampleend=clock();
			sampledur[i]=sampleend-samplestart;
			samplestart=clock();
		}
		//print output
		for (i=0;i<ARRAY_SIZE(sequentialinputs);i++){
			printf("In %d\t%d", i,out[i]);
			printf("\t%d\n", (int)sampledur[i]);

		}
		printf("\n");
		//wait until sampling period is over
		int waiting;
		waiting=1;
		int total_t;

		while (waiting){
			end_t = clock();
			total_t = (int)(end_t - start_t);
			if (total_t>=clockspersample){
				waiting=0;
				}
			}
	}

	//close(fd);
}
