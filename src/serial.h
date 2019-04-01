#include <time.h>
#include <stdint.h>
#include <pthread.h>

class SerialTimeout {

public:
    SerialTimeout(uint32_t milliseconds);

    /* Return how many milliseconds remain */
    int64_t remaining();

private:
    struct timespec expiry;
    static struct timespec timespce_now();
};

class Serial {

public:
    Serial();
    ~Serial();
    /* Default 9600,8,N,1 100ms timeout */
    Serial(const char *name, uint32_t baudrate = 9600, uint8_t bytesize = 8, char parity = 'N', uint8_t stopbits = 1, uint32_t timeout = 100);

    /* Opend */
    bool isOpend() const {return m_isOpend;}

    /* Wait until readable */
    bool waitReadable(uint32_t timeout);

    /* Read / write */
    ssize_t write(const uint8_t *buf, size_t count);

    /* Read until count bytes read or timeout expired */
    ssize_t read(uint8_t *buf, size_t count, uint32_t timeout);

    /* Set serial port comm attribute */
    int setAttribute(uint32_t baudrate, uint8_t bytesize, char parity, uint8_t stopbits);

private:
    int m_serial;
    bool m_isOpend;
    uint32_t m_timeout;

    bool errorCheck();
    pthread_mutex_t m_readLock;
    pthread_mutex_t m_writeLock;

    /* Disable copy construct */
    Serial(const Serial &);
    Serial &operator=(const Serial &);
};
