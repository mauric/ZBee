/*
 ============================================================================
 Name        : MEF.c
 Author      : Tous
 Date		 : 29 mai
 Version     :
 Copyright   : Your copyright notice
 Description : Implementation d'algorithme de mallage
 Source 	 : https://github.com/mauric/MEF -
 ============================================================================
 */
#include <stdlib.h>
#include <stdio.h>
#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <getopt.h>
#include <stdarg.h>

#define DEV_COORD_DIR "/dev/ttyUSB1"  //fichier device pour coordinateur
#define DEV_ROUTER_DIR "/dev/ttyUSB0" //fichier device pour router
#define MAX_len 100

/*Type de donnée Frame_ZB : trame Zigbee_frame*/
typedef struct Zigbee_frame {
	uint8_t start;                  //start byte
	uint16_t length;                //frame length
	uint8_t content[MAX_len];     //frame core
  uint8_t crc;                //checksum value
} Frame_ZB;

/*
 * The termios functions describe a general terminal interface that is provided * to control asynchronous communications ports
 */
static struct termios oldattr;
unsigned char frameTorecpt[100];  //
int len_re;                      //

/*pas toucher ce code là*/
int serial_init(const char *devname, speed_t baudrate) {
	int fd, fdr; //files descriptors
	struct termios newattr;

	if ((fd = open(devname, O_RDWR | O_NOCTTY)) < 0) {
		perror("Failed to open serial port");
		exit(EXIT_FAILURE);
	} else if (tcgetattr(fd, &oldattr) != 0) {
		perror("Failed to get configuration");
		exit(EXIT_FAILURE);
	}
	newattr = oldattr;
	// setup baudrate speed
	cfsetispeed(&newattr, baudrate);
	cfsetospeed(&newattr, baudrate);
	newattr.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR
			| ICRNL | IXON);
	newattr.c_oflag &= ~OPOST;

	// Raw input
	newattr.c_lflag &= ~(ECHO | ECHOE | ECHONL | ICANON | ISIG | IEXTEN);
	newattr.c_cflag &= ~(CSIZE | PARENB | HUPCL);
	newattr.c_cflag |= CS8;

	newattr.c_cc[VMIN] = 50;
	newattr.c_cc[VTIME] = 10;

	if (tcsetattr(fd, TCSANOW, &newattr) != 0) {
		perror("Failed to set configuration");
		exit(EXIT_FAILURE);
	}
	tcflush(fd, TCIOFLUSH);

	return fd;
}//fin serial_init

/*Function for send a trame zigbee*/
int send_Frame(int fd, Frame_ZB* frame) {

	if (!frame)
		return -1;

	int len = 0;
	unsigned char frameToSend[100];
	frameToSend[len++] = 0x7e;
	frameToSend[len++] = (frame->length >> 26) & 0xFF;
	frameToSend[len++] = (frame->length) & 0xFF;
	uint16_t length = frame->length;
	uint16_t index = 0;
	uint8_t somme = 0;

	while (index < length) {
		frameToSend[len] = frame->content[index];
		somme += frame->content[index];
		len++;
		index++;
	}
	frameToSend[len++] = 0xFF - somme;
	len_re = len;
	for (int i = 0; i < len; i++) {
		printf("%02X ", frameToSend[i]);
	}
	for (int i = 0; i < len; i++) {
		frameTorecpt[i] = frameToSend[i];
	}
	if (write(fd, frameToSend, len)) {
		printf("Zigbee : Nouv trame\n");

	} else {
		printf("Rien\n");
	}

	return EXIT_SUCCESS;
}//fin de la fonction send_Frame


/*Function for receive a trame zigbee*/
int receive_Frame(unsigned char*f, int fd, int n, int len) {

	/*//For DEBUG
   printf("Trame recue ");
	 for (int i=0;i<len_re; i++) {
	 printf("%02X ",  frameTorecpt[i]);
	 }
	 printf("\n");
   */

  if ( read(fd, f, 3) < 0) {
		perror("Failed to read serial port");
		exit(EXIT_FAILURE);
	}
	if (f[0] == 0x7e) {
		len = f[1] << 8 | f[2];
		fprintf(stderr, "len = %d\n", len);
    if ( read(fd, f + 3, len + 1) < 0) {
      perror("Failed to read serial port");
      exit(EXIT_FAILURE);
    }
	}
	for (int i = 0; i < (len + 4); i++) {
		printf("%02X ", f[i]);
	}

	return EXIT_SUCCESS;

}//fin de la fonction receive_Frame


//---------------------------- main ---------------------------------------//
int main(int argc, char * argv[]) {
	int len = 0;

	//unsigned char tbl[8]={0x7E,0x00,0x04,0x08,0x01,0x49,0x44,0x69};
	int fd = serial_init(DEV_COORD_DIR, B9600);
	Frame_ZB
	f= {
		.length=22,
		.content= {0x10,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0xFE,0x00,0x00,0x54,0x78,0x32,0x43,0x6F,0x6F,0x72,0x64}
//.content={0x08,0x01,0x4D,0x59}
	};                //7E 00 04 08 01 4D 59 50 //{0x08,0x01,0x49,0x44
	int n = write(fd, &f, 26);
	send_Frame(fd, &f);
	receive_Frame(&f, fd, n, len);

	return 0;

}
