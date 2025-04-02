#ifndef SERIAL_SLIP_H
#define SERIAL_SLIP_H

#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <pthread.h>
#include "log.h"

#ifndef SERIAL_SLIP_LOG_LEVEL
#define SERIAL_SLIP_LOG_LEVEL LOG_ERROR
#endif

#define SS_LOG_ERROR(fmt, ...)   LOG_MODULE(SERIAL_SLIP_LOG_LEVEL, LOG_ERROR, fmt, ##__VA_ARGS__)
#define SS_LOG_WARN(fmt, ...)    LOG_MODULE(SERIAL_SLIP_LOG_LEVEL, LOG_WARN,  fmt, ##__VA_ARGS__)
#define SS_LOG_INFO(fmt, ...)    LOG_MODULE(SERIAL_SLIP_LOG_LEVEL, LOG_INFO,  fmt, ##__VA_ARGS__)
#define SS_LOG_DEBUG(fmt, ...)   LOG_MODULE(SERIAL_SLIP_LOG_LEVEL, LOG_DEBUG, fmt, ##__VA_ARGS__)
#define SS_LOG_TRACE(fmt, ...)   LOG_MODULE(SERIAL_SLIP_LOG_LEVEL, LOG_TRACE, fmt, ##__VA_ARGS__)
#define SS_LOG_FATAL(fmt, ...)   LOG_MODULE(SERIAL_SLIP_LOG_LEVEL, LOG_FATAL, fmt, ##__VA_ARGS__)



// SLIP definitions and buffer sizes
#define SLIP_END      0xC0
#define SLIP_ESC      0xDB
#define SLIP_ESC_END  0xDC
#define SLIP_ESC_ESC  0xDD

#define RX_BUFFER_SIZE (4096 * 4)
#define TX_BUFFER_SIZE (4096 * 4)

typedef enum {
    SS_OK = 0,
    SS_ERR_BUFFER_TOO_SMALL,
    SS_ERR_BUFFER_OVERFLOW,
    SS_ERR_ESCAPE_OVERFLOW,
    SS_ERR_CRC_MISMATCH,
    SS_ERR_MEMORY,
    SS_ERR_PORT_OPEN,
    SS_ERR_COMM_STATE,
    SS_ERR_SET_COMM_STATE,
    SS_ERR_CREATE_EVENT,
    SS_ERR_CREATE_THREAD,
    SS_ERR_READ,
    SS_ERR_WRITE,
    SS_ERR_GET_MSG_TIMEOUT,
    SS_ERR_DECODE,
    SS_ERR_MAX,
} SerialSlipError;

// Internal message node for the message queue.
typedef struct MessageNode {
    uint8_t *data;
    size_t length;
    struct MessageNode *next;
} MessageNode;

// Definition of the SerialSlip structure.
typedef struct SerialSlip {
    int fd;                  // deskryptor portu szeregowego
    pthread_t thread;        // wątek odczytujący
    volatile bool stopThread;
    volatile bool connected;
    char portName[64];

    pthread_mutex_t writeLock;
    pthread_mutex_t queueLock;
    pthread_cond_t  msgCond; // zmienna warunkowa do oczekiwania na wiadomość
    MessageNode *msgHead;
    MessageNode *msgTail;

    uint8_t rxBuffer[RX_BUFFER_SIZE];
    size_t rxBufferLength;
} SerialSlip;

SerialSlip *serial_slip_open(const char *comPort);
void serial_slip_close(SerialSlip *ctx);
SerialSlipError serial_slip_write(SerialSlip *ctx, const uint8_t *data, size_t len);
SerialSlipError serial_slip_get_message(SerialSlip *ctx, uint8_t *buffer, size_t bufferSize, size_t *msgLen, unsigned long timeout);
SerialSlipError serial_slip_write_and_get_response(SerialSlip *ctx, const uint8_t *data, size_t len, uint8_t *responseBuffer, size_t responseBufferSize, size_t *responseLen, unsigned long timeout, int max_retries);
const char *serial_slip_decode_error(SerialSlipError err);
#ifdef __cplusplus
}
#endif
#endif // SERIAL_SLIP_H
