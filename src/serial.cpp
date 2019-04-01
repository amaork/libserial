#include <errno.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include "serial.h"

SerialTimeout::SerialTimeout(uint32_t milliseconds) : expiry(timespce_now()) {

    /* Unified convert to nanoseconds */
    int64_t tv_nsec = expiry.tv_nsec + (milliseconds * 1e6);

    /* More than 1 second */
    if (tv_nsec > 1e9) {

        int64_t sec_diff = tv_nsec / static_cast<int>(1e9);
        expiry.tv_nsec = tv_nsec % static_cast<int>(1e9);
        expiry.tv_sec += sec_diff;
    }
    else {

        expiry.tv_nsec = tv_nsec;
    }
}

struct timespec SerialTimeout::timespce_now() {

    struct timespec time;
    clock_gettime(CLOCK_MONOTONIC, &time);
    return time;
}

int64_t SerialTimeout::remaining() {

    timespec now = timespce_now();
    int64_t milliseconds = (expiry.tv_sec - now.tv_sec) * 1e3;
    milliseconds += (expiry.tv_nsec - now.tv_nsec) / 1e6;
    return milliseconds;
}

Serial::Serial(const char *name, uint32_t baudrate, uint8_t bytesize, char parity, uint8_t stopbits, uint32_t timeout) {

    m_isOpend = false;
    m_timeout = timeout;

    /* First initlize mutex lock */
    pthread_mutex_init(&m_readLock, NULL);
    pthread_mutex_init(&m_writeLock, NULL);

    /* Open serial port */
    if ((m_serial = ::open(name, O_RDWR | O_NOCTTY | O_NONBLOCK)) == -1) {

        fprintf(stdout, "Open serial port[%s] error: %s\n", name, strerror(errno));
        return;
    }

    /* Set serial port attribute */
    if (setAttribute(baudrate, bytesize, parity, stopbits)) {

        return;
    }

    m_isOpend = true;
}

Serial::~Serial() {

    if (isOpend() && m_serial != -1) {

        close(m_serial);
        m_serial = -1;
        m_isOpend = false;
    }

    pthread_mutex_destroy(&m_readLock);
    pthread_mutex_destroy(&m_writeLock);
    fprintf(stdout, "Serial close, exit\n");
}

bool Serial::errorCheck() {

    if (!isOpend()) {

        fprintf(stderr, "Please open port first!!!\n");
        return true;
    }

    return false;
}

int Serial::setAttribute(uint32_t baudrate, uint8_t bytesize, char parity, uint8_t stopbits) {

    struct termios new_attr, old_attr;

    /* Get current serial port settings	*/
    if  ( tcgetattr(m_serial, &old_attr)  !=  0) {
        perror("Get serial port settings error");
        return -1;
    }

    memset(&new_attr, 0, sizeof(new_attr));
    /* Ignore mode line and enable receive device */
    new_attr.c_cflag |=  CLOCAL | CREAD;
    /* Clear data bit length	*/
    new_attr.c_cflag &= ~CSIZE;

    /* Settings data valid bits	*/
    switch (bytesize) {
        case 5:
            new_attr.c_cflag |= CS5;
            break;

        case 6:
            new_attr.c_cflag |= CS6;
            break;

        case 7:
            new_attr.c_cflag |= CS7;
            break;

        case 8:
            new_attr.c_cflag |= CS8;
            break;

        default:
            fprintf(stderr, "Unknown bytesize[%d], valid:[5/6/7/8]\n", bytesize);
            return -1;
    }

    /* Settings data parity	*/
    switch (parity) {
        case 'O':
            new_attr.c_cflag |= PARENB;
            new_attr.c_cflag |= PARODD;
            new_attr.c_iflag |= (INPCK | ISTRIP);
            break;

        case 'E':
            new_attr.c_cflag |= PARENB;
            new_attr.c_cflag &= ~PARODD;
            new_attr.c_iflag |= (INPCK | ISTRIP);
            break;

        case 'N':
            new_attr.c_cflag &= ~PARENB;
            break;

        default:
            fprintf(stderr, "Unknown parity[%c], valid:[O/E/N]\n", parity);
            break;
    }

    /* Settings baud rate */
    switch (baudrate) {
        case 2400:
            cfsetispeed(&new_attr, B2400);
            cfsetospeed(&new_attr, B2400);
            break;

        case 4800:
            cfsetispeed(&new_attr, B4800);
            cfsetospeed(&new_attr, B4800);
            break;

        case 9600:
            cfsetispeed(&new_attr, B9600);
            cfsetospeed(&new_attr, B9600);
            break;

        case 19200:
            cfsetispeed(&new_attr, B19200);
            cfsetospeed(&new_attr, B19200);
            break;

        case 38400:
            cfsetispeed(&new_attr, B38400);
            cfsetospeed(&new_attr, B38400);
            break;

        case 57600:
            cfsetispeed(&new_attr, B57600);
            cfsetospeed(&new_attr, B57600);
            break;

        case 115200:
            cfsetispeed(&new_attr, B115200);
            cfsetospeed(&new_attr, B115200);
            break;

        case 460800:
            cfsetispeed(&new_attr, B460800);
            cfsetospeed(&new_attr, B460800);
            break;

        default:
            fprintf(stderr, "Unknown baudate[%d]\n", baudrate);
            break;
    }

    /* Setting comm stop bit number */
    switch (stopbits) {
        case 1:
            new_attr.c_cflag &= ~CSTOPB;
            break;

        case 2:
            new_attr.c_cflag |= CSTOPB;
            break;

        default:
            fprintf(stderr, "Unknown stopbit[%d], valid:[1/2]\n", stopbits);
            break;
    }

    /* Enable standard input and echo character, and enable terminal signal such as ctrl^c */
    new_attr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
    new_attr.c_oflag &= ~OPOST;   /*Output*/

    /* XXX VTIME and VMIN will influence serial port block mode, if the device open it as O_NONBLOCK mode VTIME and VMIN will invalid */
    /* Set read minmum read size and timeout value	*/
    new_attr.c_cc[VTIME] = 0; 	/* Byte and byte timeout is 0s, when first byte is read  */
    new_attr.c_cc[VMIN] = 0;    /* At least have 0 byte data then return or will block call read process */

    /* Flush output queue */
    tcflush(m_serial, TCIFLUSH);

    /* Settings tty attribute and make it works implite*/
    if ((tcsetattr(m_serial, TCSANOW, &new_attr)) != 0) {
        perror("tcsetattr:");
        return -1;
    }

    return 0;
}

ssize_t Serial::write(const uint8_t *buf, size_t count) {

    if (errorCheck()) {

        return -1;
    }

    ssize_t ret;
    size_t bytes_write;
    pthread_mutex_lock(&m_writeLock);

    bytes_write = 0;

    while (bytes_write < count) {

        if ((ret = ::write(m_serial, buf + bytes_write, count - bytes_write)) == -1) {

            perror("Write error");
            break;
        }

        bytes_write += ret;
    }

    pthread_mutex_unlock(&m_writeLock);

    return bytes_write;
}

bool Serial::waitReadable(uint32_t timeout) {

    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(m_serial, &readfds);

    struct timespec timeout_ts;
    timeout_ts.tv_sec = timeout / 1e3;
    timeout_ts.tv_nsec = (timeout - (timeout_ts.tv_sec * 1e3)) * 1e6;

    /* Setup select, it will blocked until serial readable(r > 0) or timeout r == 0 */
    int r = pselect(m_serial + 1, &readfds, NULL, NULL, &timeout_ts, NULL);

    /* Error */
    if (r < 0) {

        perror("WaitReadable");
        return false;
    }

    /* Timeout */
    if (r == 0) {

#ifdef _DEBUG_SERIAL_
        fprintf(stderr, "Read timeout, remain[%dms]\n", timeout);
#endif
        return false;
    }

    return true;
}

ssize_t Serial::read(uint8_t *buf, size_t count, uint32_t timeout) {

    if (errorCheck()) {

        return -1;
    }

    size_t bytes_read;
    pthread_mutex_lock(&m_readLock);

    ssize_t ret;
    int64_t timeout_remain_ms;
    SerialTimeout toal_timeout = SerialTimeout(timeout);

    /* First check if already has data */
    ret = ::read(m_serial, buf, count);
    bytes_read = (ret > 0) ? ret : 0;

    while (bytes_read < count) {

        /*  Check timeout */
        if ((timeout_remain_ms = toal_timeout.remaining()) <= 0) {

            break;
        }

        /* Pselect block wait readable */
        if (!waitReadable(timeout_remain_ms)) {

            continue;
        }

        /* Run to here means atleast has 1 byte readable */
        ret = ::read(m_serial, buf + bytes_read, count - bytes_read);

#ifdef _DEBUG_SERIAL_
        fprintf(stdout, "R:%d,%d,%d,%lld\n", ret, bytes_read, count, timeout_remain_ms);
#endif

        /* Read error */
        if (ret == -1) {

            break;
        }

        bytes_read += ret;
    }

    pthread_mutex_unlock(&m_readLock);
    return bytes_read;
}
