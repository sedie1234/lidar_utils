/*
 * uart.c
 *
 * @date 2019/08/09
 * @author Cosmin Tanislav
 * @author Cristian Fatu
 */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <limits.h>

#include "uart_util.h"

/*
* Start the UART device.
*
* @param dev points to the UART device to be started, must have filename and rate populated
* @param canonical whether to define some compatibility flags for a canonical interface
*
* @return - 0 if the starting procedure succeeded
*         - negative if the starting procedure failed
*/
int uart_start(struct UartDevice* dev, bool canonical) {
    struct termios *tty;
    int fd;
    int rc;

    fd = open(dev->filename, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        printf("%s: failed to open UART device\r\n", __func__);
        return fd;
    }

    tty = (termios *)malloc(sizeof(*tty));
    if (!tty) {
        printf("%s: failed to allocate UART TTY instance\r\n", __func__);
        return -ENOMEM;
    }

    memset(tty, 0, sizeof(*tty));

    /*
    * Set baud-rate.
    */
    tty->c_cflag |= dev->rate;

    /* Ignore framing and parity errors in input. */
    tty->c_iflag |=  IGNPAR;

    /* Use 8-bit characters. This too may affect standard streams,
    * but any sane C library can deal with 8-bit characters. */
    tty->c_cflag |=  CS8;

    /* Enable receiver. */
    tty->c_cflag |=  CREAD;

    if (canonical) {
        /* Enable canonical mode.
        * This is the most important bit, as it enables line buffering etc. */
        tty->c_lflag |= ICANON;
    } else {
        /* To maintain best compatibility with normal behaviour of terminals,
        * we set TIME=0 and MAX=1 in noncanonical mode. This means that
        * read() will block until at least one byte is available. */
        tty->c_cc[VTIME] = 0;
        tty->c_cc[VMIN] = 1;
    }

    /*
    * Flush port.
    */
    tcflush(fd, TCIFLUSH);

    /*
    * Apply attributes.
    */
    rc = tcsetattr(fd, TCSANOW, tty);
    if (rc) {
        printf("%s: failed to set attributes\r\n", __func__);
        return rc;
    }

    dev->fd = fd;
    dev->tty = tty;

    return 0;
}

/*
* Read a string from the UART device.
*
* @param dev points to the UART device to be read from
* @param buf points to the start of buffer to be read into
* @param buf_len length of the buffer to be read
*
* @return - number of bytes read if the read procedure succeeded
*         - negative if the read procedure failed
*/
int uart_reads(struct UartDevice* dev, char *buf, size_t buf_len) {
    int rc;
    size_t total_read = 0;

    while(total_read < buf_len - 1) {
	rc = read(dev->fd, buf + total_read, buf_len - 1 - total_read);
	// printf("Read return code(rc): %d\n", rc);

	if(rc < 0) {
		printf("%s: failed to read uart data\r\n", __func__);
		return rc;
	}

	total_read += rc;

	for(int i = total_read - rc; i < total_read; i++) {
		if(buf[i] == '\n') {
			buf[i] = '\0';
			return total_read;
		}
	}
    }

    buf[total_read] = '\0';

    //rc = read(dev->fd, buf, buf_len - 1);
    //printf("Read return code(rc): %d\n", rc);

    //if (rc < 0) {
    //    printf("%s: failed to read uart data\r\n", __func__);
    //    return rc;
    //}

    //for(int i = 0; i < rc; i++) {
	//if(buf[i] == '\n') {
	//	buf[i] = '\0';
	//	rc = i + 1;
	//	break;
	//}
    //}

    //buf[rc] = '\0';
    //return rc;
    return total_read;
}

/*
* Write data to the UART device.
*
* @param dev points to the UART device to be written to
* @param buf points to the start of buffer to be written from
* @param buf_len length of the buffer to be written
*
* @return - number of bytes written if the write procedure succeeded
*         - negative if the write procedure failed
*/
int uart_writen(struct UartDevice* dev, char *buf, size_t buf_len) {
    return write(dev->fd, buf, buf_len);
}

/*
* Write a string to the UART device.
*
* @param dev points to the UART device to be written to
* @param string points to the start of buffer to be written from
*
* @return - number of bytes written if the write procedure succeeded
*         - negative if the write procedure failed
*/
int uart_writes(struct UartDevice* dev, char *string) {
    size_t len = strlen(string);
    return uart_writen(dev, string, len);
}

/*
* Stop the UART device.
*
* @param dev points to the UART device to be stopped
*/
void uart_stop(struct UartDevice* dev) {
    free(dev->tty);
}