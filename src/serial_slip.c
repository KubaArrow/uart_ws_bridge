#include "uart_ws_bridge/serial_slip.h"
#include "uart_ws_bridge/log.h"
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>      // close(), usleep()
#include <fcntl.h>       // open(), O_RDWR, O_NOCTTY
#include <errno.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <pthread.h>
#include <time.h>

// CRC8 lookup table (polynomial 0x07)
static const uint8_t CRC8_TABLE[256] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15,
    0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
    0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65,
    0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
    0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5,
    0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85,
    0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
    0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2,
    0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
    0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2,
    0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32,
    0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
    0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42,
    0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
    0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c,
    0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec,
    0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
    0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c,
    0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
    0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c,
    0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b,
    0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
    0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b,
    0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
    0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb,
    0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb,
    0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3
};

static const char *SerialSlipErrors[SS_ERR_MAX] = {
    "SS_OK",
    "SS_ERR_BUFFER_TOO_SMALL",
    "SS_ERR_BUFFER_OVERFLOW",
    "SS_ERR_ESCAPE_OVERFLOW",
    "SS_ERR_CRC_MISMATCH",
    "SS_ERR_MEMORY",
    "SS_ERR_PORT_OPEN",
    "SS_ERR_COMM_STATE",
    "SS_ERR_SET_COMM_STATE",
    "SS_ERR_CREATE_EVENT",
    "SS_ERR_CREATE_THREAD",
    "SS_ERR_READ",
    "SS_ERR_WRITE",
    "SS_ERR_GET_MSG_TIMEOUT",
    "SS_ERR_DECODE"
};

// Uwaga: w tej wersji zakładamy, że struktura SerialSlip jest zdefiniowana np. tak:
/*
typedef struct MessageNode {
    uint8_t *data;
    size_t length;
    struct MessageNode *next;
} MessageNode;

typedef struct SerialSlip {
    int fd;                     // deskryptor portu szeregowego
    char portName[64];          // nazwa portu, np. "/dev/ttyUSB0"
    bool connected;
    pthread_mutex_t writeLock;
    pthread_mutex_t queueLock;
    pthread_cond_t  msgCond;    // zmienna warunkowa do oczekiwania na wiadomość
    MessageNode *msgHead;
    MessageNode *msgTail;
    uint8_t rxBuffer[RX_BUFFER_SIZE];
    size_t rxBufferLength;
    bool stopThread;
    pthread_t thread;
} SerialSlip;
*/

//--------------------------------------------------------------------------
// Obliczenie CRC8 dla podanych danych.
static uint8_t crc8(const uint8_t *data, size_t len)
{
    SS_LOG_DEBUG("Calculating CRC for %zu bytes\n", len);
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc = CRC8_TABLE[crc ^ data[i]];
    }
    SS_LOG_DEBUG("CRC result: 0x%02X\n", crc);
    return crc;
}

//--------------------------------------------------------------------------
// Sprawdzenie, czy port jest połączony.
// W tej wersji sprawdzamy, czy deskryptor jest poprawny oraz czy linia DTR jest ustawiona.
static bool serial_is_connected(SerialSlip *ctx)
{
    if (ctx->fd < 0) {
        ctx->connected = false;
        return false;
    }
    int status;
    if (ioctl(ctx->fd, TIOCMGET, &status) < 0) {
        ctx->connected = false;
        return false;
    }
    // Sprawdzamy, czy flaga DTR jest ustawiona
    if (status & TIOCM_DTR) {
        return true;
    }
    ctx->connected = false;
    return false;
}

//--------------------------------------------------------------------------
// SLIP encode: kodowanie danych wejściowych i zapis wyniku do bufora 'out'.
// W przypadku powodzenia ustaw *encoded_len i zwróć SS_OK, w przeciwnym razie kod błędu.
static SerialSlipError slip_encode(const uint8_t *data, size_t len, uint8_t *out, size_t maxEncodedSize, size_t *encoded_len)
{
    SS_LOG_DEBUG("SLIP encode started. Input len: %zu, Max output: %zu\n", len, maxEncodedSize);
    size_t pos = 0;
    if (maxEncodedSize < 2) {
        SS_LOG_ERROR("Fatal: Buffer too small (min 2 bytes required)\n");
        return SS_ERR_BUFFER_TOO_SMALL;
    }
    out[pos++] = SLIP_END;
    for (size_t i = 0; i < len; i++) {
        if (pos >= maxEncodedSize) {
            SS_LOG_ERROR("Buffer overflow during data byte %zu/%zu\n", i + 1, len);
            return SS_ERR_BUFFER_OVERFLOW;
        }
        if (data[i] == SLIP_END) {
            if (pos + 2 > maxEncodedSize) {
                SS_LOG_ERROR("Escape overflow at byte %zu\n", i);
                return SS_ERR_ESCAPE_OVERFLOW;
            }
            out[pos++] = SLIP_ESC;
            out[pos++] = SLIP_ESC_END;
        } else if (data[i] == SLIP_ESC) {
            if (pos + 2 > maxEncodedSize) {
                SS_LOG_ERROR("Escape overflow at byte %zu\n", i);
                return SS_ERR_ESCAPE_OVERFLOW;
            }
            out[pos++] = SLIP_ESC;
            out[pos++] = SLIP_ESC_ESC;
        } else {
            out[pos++] = data[i];
        }
    }
    uint8_t crc = crc8(data, len);
    SS_LOG_DEBUG("Appending CRC: 0x%02X\n", crc);
    if (crc == SLIP_END) {
        if (pos + 2 > maxEncodedSize) {
            SS_LOG_ERROR("CRC escape overflow (END case)\n");
            return SS_ERR_ESCAPE_OVERFLOW;
        }
        out[pos++] = SLIP_ESC;
        out[pos++] = SLIP_ESC_END;
    } else if (crc == SLIP_ESC) {
        if (pos + 2 > maxEncodedSize) {
            SS_LOG_ERROR("CRC escape overflow (ESC case)\n");
            return SS_ERR_ESCAPE_OVERFLOW;
        }
        out[pos++] = SLIP_ESC;
        out[pos++] = SLIP_ESC_ESC;
    } else {
        if (pos >= maxEncodedSize) {
            SS_LOG_ERROR("CRC overflow\n");
            return SS_ERR_BUFFER_OVERFLOW;
        }
        out[pos++] = crc;
    }
    if (pos >= maxEncodedSize) {
        SS_LOG_ERROR("Final END byte overflow\n");
        return SS_ERR_BUFFER_OVERFLOW;
    }
    out[pos++] = SLIP_END;
    SS_LOG_DEBUG("SLIP encode successful. Total encoded: %zu bytes\n", pos);
    *encoded_len = pos;
    return SS_OK;
}

//--------------------------------------------------------------------------
// SLIP decode: dekodowanie pakietu SLIP i zapis danych (bez CRC) do 'decoded'.
// W przypadku powodzenia ustaw *decoded_len i zwróć SS_OK, w przeciwnym razie kod błędu.
static SerialSlipError slip_decode(const uint8_t *packet, size_t packet_len, uint8_t *decoded, size_t *decoded_len)
{
    SS_LOG_DEBUG("SLIP decode started. Packet length: %zu\n", packet_len);
    if (packet_len < 3) {
        SS_LOG_ERROR("Invalid packet: Too short (%zu bytes)\n", packet_len);
        return SS_ERR_DECODE;
    }
    size_t j = 0;
    bool escape = false;
    for (size_t i = 1; i < packet_len - 1; i++) {
        uint8_t byte = packet[i];
        if (escape) {
            if (byte == SLIP_ESC_END) {
                decoded[j++] = SLIP_END;
            } else if (byte == SLIP_ESC_ESC) {
                decoded[j++] = SLIP_ESC;
            } else {
                SS_LOG_ERROR("Decode error: Unexpected escape sequence: 0x%02X\n", byte);
                return SS_ERR_DECODE;
            }
            escape = false;
        } else {
            if (byte == SLIP_ESC) {
                escape = true;
            } else {
                decoded[j++] = byte;
            }
        }
    }
    if (j < 1) {
        SS_LOG_ERROR("Decode failed: No data after unescaping\n");
        return SS_ERR_DECODE;
    }
    uint8_t received_crc = decoded[j - 1];
    uint8_t calc_crc = crc8(decoded, j - 1);
    SS_LOG_DEBUG("CRC Check: Received=0x%02X, Calculated=0x%02X\n", received_crc, calc_crc);
    if (received_crc != calc_crc) {
        SS_LOG_ERROR("CRC mismatch\n");
        return SS_ERR_CRC_MISMATCH;
    }
    *decoded_len = j - 1;
    SS_LOG_DEBUG("SLIP decode successful. Data length: %zu bytes\n", *decoded_len);
    return SS_OK;
}

//--------------------------------------------------------------------------
// Funkcja pomocnicza do pobierania wiadomości – wersja, która zakłada, że kolejka jest zablokowana.
static bool pop_message_locked(SerialSlip *ctx, uint8_t **data, size_t *length)
{
    if (ctx->msgHead) {
        MessageNode *node = ctx->msgHead;
        *data = node->data;
        *length = node->length;
        ctx->msgHead = node->next;
        if (!ctx->msgHead) {
            ctx->msgTail = NULL;
        }
        free(node);
        return true;
    }
    return false;
}

//--------------------------------------------------------------------------
// Wrzucenie nowej wiadomości do kolejki.
static SerialSlipError push_message(SerialSlip *ctx, const uint8_t *data, size_t length)
{
    SS_LOG_DEBUG("Pushing message. Length: %zu bytes\n", length);
    MessageNode *node = (MessageNode *)malloc(sizeof(MessageNode));
    if (!node) {
        SS_LOG_ERROR("Memory error: Failed to allocate MessageNode\n");
        return SS_ERR_MEMORY;
    }
    node->data = (uint8_t *)malloc(length);
    if (!node->data) {
        SS_LOG_ERROR("Memory error: Failed to allocate data buffer (%zu bytes)\n", length);
        free(node);
        return SS_ERR_MEMORY;
    }
    memcpy(node->data, data, length);
    node->length = length;
    node->next = NULL;

    pthread_mutex_lock(&ctx->queueLock);
    if (ctx->msgTail) {
        ctx->msgTail->next = node;
        ctx->msgTail = node;
    } else {
        ctx->msgHead = ctx->msgTail = node;
    }
    pthread_cond_signal(&ctx->msgCond); // sygnalizujemy pojawienie się wiadomości
    pthread_mutex_unlock(&ctx->queueLock);
    return SS_OK;
}

//--------------------------------------------------------------------------
// Pobranie wiadomości z kolejki (funkcja blokująca; nie używana przy oczekiwaniu z timeoutem).
static bool pop_message(SerialSlip *ctx, uint8_t **data, size_t *length)
{
    bool result = false;
    pthread_mutex_lock(&ctx->queueLock);
    result = pop_message_locked(ctx, data, length);
    pthread_mutex_unlock(&ctx->queueLock);
    return result;
}

//--------------------------------------------------------------------------
// Przetwarzanie fragmentu odebranych danych: dołączenie do bufora RX, wyodrębnienie kompletnych pakietów SLIP,
// a następnie próba ich dekodowania.
static void process_chunk(SerialSlip *ctx, const uint8_t *data, size_t len)
{
    SS_LOG_DEBUG("Processing chunk: %zu bytes, buffer %zu/%d\n", len, ctx->rxBufferLength, RX_BUFFER_SIZE);
    if (ctx->rxBufferLength + len > RX_BUFFER_SIZE) {
        SS_LOG_ERROR("Buffer overflow! Resetting RX buffer\n");
        ctx->rxBufferLength = 0;
    }
    memcpy(ctx->rxBuffer + ctx->rxBufferLength, data, len);
    ctx->rxBufferLength += len;
    while (true) {
        size_t start_index = 0;
        while (start_index < ctx->rxBufferLength && ctx->rxBuffer[start_index] != SLIP_END) {
            start_index++;
        }
        if (start_index >= ctx->rxBufferLength) {
            SS_LOG_DEBUG("No start byte found. Buffer length: %zu\n", ctx->rxBufferLength);
            break;
        }
        size_t end_index = start_index + 1;
        while (end_index < ctx->rxBufferLength && ctx->rxBuffer[end_index] != SLIP_END) {
            end_index++;
        }
        if (end_index >= ctx->rxBufferLength) {
            SS_LOG_DEBUG("No end byte found. Waiting for more data\n");
            break;
        }
        size_t packet_len = end_index - start_index + 1;
        if (packet_len < 4) {
            SS_LOG_DEBUG("Incomplete packet (only %zu bytes). Waiting for more data.\n", packet_len);
            size_t new_length = ctx->rxBufferLength - end_index;
            memmove(ctx->rxBuffer, ctx->rxBuffer + end_index, new_length);
            ctx->rxBufferLength = new_length;
            break;
        }
        uint8_t packet[RX_BUFFER_SIZE];
        if (packet_len > sizeof(packet)) {
            SS_LOG_ERROR("Packet too big (%zu > %zu). Truncating\n", packet_len, sizeof(packet));
            packet_len = sizeof(packet);
        }
        memcpy(packet, ctx->rxBuffer + start_index, packet_len);
        size_t remaining = ctx->rxBufferLength - (end_index + 1);
        memmove(ctx->rxBuffer, ctx->rxBuffer + end_index + 1, remaining);
        ctx->rxBufferLength = remaining;
        uint8_t decoded[RX_BUFFER_SIZE];
        size_t decoded_len = 0;
        SerialSlipError err = slip_decode(packet, packet_len, decoded, &decoded_len);
        if (err == SS_OK) {
            err = push_message(ctx, decoded, decoded_len);
            if (err != SS_OK) {
                SS_LOG_ERROR("Failed to push message, error code: %s\n", serial_slip_decode_error(err));
            }
        } else {
            SS_LOG_ERROR("Decode failed! Packet dropped. Error code: %s\n", serial_slip_decode_error(err));
        }
    }
}

//--------------------------------------------------------------------------
// Próba ponownego połączenia: zamknięcie starego deskryptora, ponowne otwarcie portu i ponowna konfiguracja.
static SerialSlipError attempt_reconnect(SerialSlip *ctx)
{
    if (ctx->fd >= 0) {
        close(ctx->fd);
    }
    // W systemie Linux nazwa portu jest zwykle podana jako np. "/dev/ttyUSB0"
    ctx->fd = open(ctx->portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (ctx->fd < 0) {
        SS_LOG_ERROR("Reconnect failed: unable to open %s (error %s)\n", ctx->portName, strerror(errno));
        return SS_ERR_PORT_OPEN;
    }
    // Konfiguracja portu przy użyciu termios
    struct termios options;
    if (tcgetattr(ctx->fd, &options) < 0) {
        SS_LOG_ERROR("Reconnect failed: unable to get termios attributes (error %s)\n", strerror(errno));
        close(ctx->fd);
        ctx->fd = -1;
        return SS_ERR_COMM_STATE;
    }
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag &= ~PARENB;      // brak parzystości
    options.c_cflag &= ~CSTOPB;      // jeden bit stopu
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;          // 8 bitów danych
    options.c_cflag |= CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    // Ustawienie trybu niekanonicznego
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 1;
    if (tcsetattr(ctx->fd, TCSANOW, &options) < 0) {
        SS_LOG_ERROR("Reconnect failed: unable to set termios attributes (error %s)\n", strerror(errno));
        close(ctx->fd);
        ctx->fd = -1;
        return SS_ERR_SET_COMM_STATE;
    }
    // Ustawienie linii DTR
    int status;
    if (ioctl(ctx->fd, TIOCMGET, &status) < 0) {
        SS_LOG_ERROR("Reconnect failed: unable to get modem status (error %s)\n", strerror(errno));
        close(ctx->fd);
        ctx->fd = -1;
        return SS_ERR_COMM_STATE;
    }
    status |= TIOCM_DTR;
    if (ioctl(ctx->fd, TIOCMSET, &status) < 0) {
        SS_LOG_ERROR("Reconnect failed: unable to set DTR (error %s)\n", strerror(errno));
        close(ctx->fd);
        ctx->fd = -1;
        return SS_ERR_SET_COMM_STATE;
    }
    SS_LOG_DEBUG("Reconnect succeeded: connected to %s successfully\n", ctx->portName);
    ctx->connected = true;
    return SS_OK;
}

//--------------------------------------------------------------------------
// Wątek czytający: ciągłe odczytywanie z portu szeregowego i przetwarzanie odebranych danych.
static void *serial_slip_read_thread(void *param)
{
    SerialSlip *ctx = (SerialSlip *)param;
    SS_LOG_DEBUG("Read thread started\n");
    uint8_t buffer[RX_BUFFER_SIZE];
    while (!ctx->stopThread) {
        if (ctx->connected) {
            ssize_t bytesRead = read(ctx->fd, buffer, sizeof(buffer));
            if (bytesRead > 0) {
                SS_LOG_DEBUG("Received %zd bytes\n", bytesRead);
                process_chunk(ctx, buffer, (size_t)bytesRead);
            } else if (bytesRead < 0) {
                // Jeśli błąd inny niż przerwanie
                if (errno != EAGAIN && errno != EINTR) {
                    SS_LOG_ERROR("Read error (error %s)\n", strerror(errno));
                }
            }
        } else {
            usleep(100 * 1000); // 100 ms
        }
    }
    SS_LOG_DEBUG("Read thread exiting\n");
    return NULL;
}

//--------------------------------------------------------------------------
// Otwarcie portu szeregowego i inicjalizacja struktury SerialSlip.
SerialSlip *serial_slip_open(const char *comPort)
{
    SS_LOG_DEBUG("Opening serial port: %s\n", comPort);
    SerialSlip *ctx = (SerialSlip *)malloc(sizeof(SerialSlip));
    if (!ctx) {
        SS_LOG_ERROR("Memory error: Failed to allocate SerialSlip\n");
        return NULL;
    }
    memset(ctx, 0, sizeof(SerialSlip));
    strncpy(ctx->portName, comPort, sizeof(ctx->portName) - 1);
    ctx->fd = open(ctx->portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (ctx->fd < 0) {
        SS_LOG_ERROR("Port error: Failed to open %s (error %s)\n", ctx->portName, strerror(errno));
        free(ctx);
        return NULL;
    }
    // Konfiguracja portu
    struct termios options;
    if (tcgetattr(ctx->fd, &options) < 0) {
        SS_LOG_ERROR("Failed to get termios attributes (error %s)\n", strerror(errno));
        close(ctx->fd);
        free(ctx);
        return NULL;
    }
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_cflag |= CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    options.c_cc[VTIME] = 0;
    options.c_cc[VMIN] = 1;
    if (tcsetattr(ctx->fd, TCSANOW, &options) < 0) {
        SS_LOG_ERROR("Failed to set termios attributes (error %s)\n", strerror(errno));
        close(ctx->fd);
        free(ctx);
        return NULL;
    }
    // Ustawienie DTR
    int status;
    if (ioctl(ctx->fd, TIOCMGET, &status) < 0) {
        SS_LOG_ERROR("Failed to get modem status (error %s)\n", strerror(errno));
        close(ctx->fd);
        free(ctx);
        return NULL;
    }
    status |= TIOCM_DTR;
    if (ioctl(ctx->fd, TIOCMSET, &status) < 0) {
        SS_LOG_ERROR("Failed to set DTR (error %s)\n", strerror(errno));
    }
    // Inicjalizacja mutexów i zmiennej warunkowej
    pthread_mutex_init(&ctx->writeLock, NULL);
    pthread_mutex_init(&ctx->queueLock, NULL);
    pthread_cond_init(&ctx->msgCond, NULL);
    ctx->rxBufferLength = 0;
    ctx->stopThread = false;
    ctx->msgHead = ctx->msgTail = NULL;
    ctx->connected = true;
    // Utworzenie wątku czytającego
    if (pthread_create(&ctx->thread, NULL, serial_slip_read_thread, ctx) != 0) {
        SS_LOG_ERROR("Thread error: Failed to create reader thread (error %s)\n", strerror(errno));
        close(ctx->fd);
        pthread_mutex_destroy(&ctx->writeLock);
        pthread_mutex_destroy(&ctx->queueLock);
        pthread_cond_destroy(&ctx->msgCond);
        free(ctx);
        return NULL;
    }
    SS_LOG_DEBUG("SerialSlip initialized successfully on port %s\n", comPort);
    return ctx;
}

//------------------------------------------------------------------------------
// Zamknięcie portu szeregowego i zwolnienie zasobów.
void serial_slip_close(SerialSlip *ctx)
{
    if (!ctx) {
        return;
    }
    ctx->stopThread = true;
    pthread_join(ctx->thread, NULL);
    if (ctx->fd >= 0) {
        close(ctx->fd);
    }
    // Zwolnienie wszystkich wiadomości w kolejce
    uint8_t *dummy;
    size_t dummyLen;
    while (pop_message(ctx, &dummy, &dummyLen)) {
        free(dummy);
    }
    pthread_mutex_destroy(&ctx->writeLock);
    pthread_mutex_destroy(&ctx->queueLock);
    pthread_cond_destroy(&ctx->msgCond);
    free(ctx);
}

//--------------------------------------------------------------------------
// Zapis danych do portu szeregowego. Przed wysłaniem sprawdzamy, czy port jest aktywny,
// a w przeciwnym razie podejmujemy próbę ponownego połączenia.
SerialSlipError serial_slip_write(SerialSlip *ctx, const uint8_t *data, size_t len)
{
    SS_LOG_DEBUG("Write start | Data length: %zu bytes\n", len);
    if (!serial_is_connected(ctx)) {
        SerialSlipError recon_err = attempt_reconnect(ctx);
        if (recon_err != SS_OK) {
            SS_LOG_ERROR("Write failed: port not connected and reconnect failed with error code %s\n", serial_slip_decode_error(recon_err));
            return recon_err;
        }
    }
    uint8_t encoded[TX_BUFFER_SIZE];
    size_t encoded_len = 0;
    SerialSlipError err = slip_encode(data, len, encoded, sizeof(encoded), &encoded_len);
    if (err != SS_OK) {
        SS_LOG_ERROR("Encode failed, error code: %s\n", serial_slip_decode_error(err));
        return err;
    }
    pthread_mutex_lock(&ctx->writeLock);
    ssize_t bytesWritten = write(ctx->fd, encoded, encoded_len);
    SS_LOG_DEBUG("Write result: %s | Bytes written: %zd/%zu\n", (bytesWritten == (ssize_t)encoded_len) ? "OK" : "FAIL", bytesWritten, encoded_len);
    pthread_mutex_unlock(&ctx->writeLock);
    if (bytesWritten != (ssize_t)encoded_len) {
        SS_LOG_ERROR("Write error: Written %zd out of %zu bytes\n", bytesWritten, encoded_len);
        return SS_ERR_WRITE;
    }
    return SS_OK;
}

//--------------------------------------------------------------------------
// Funkcja blokująca oczekująca na przyjście wiadomości (timeout w ms).
// W przypadku powodzenia kopiuje do bufora do bufferSize bajtów, ustawia *msgLen i zwraca SS_OK.
SerialSlipError serial_slip_get_message(SerialSlip *ctx, uint8_t *buffer, size_t bufferSize, size_t *msgLen, unsigned long timeout)
{
    SS_LOG_DEBUG("Waiting for message | Timeout: %lu ms\n", timeout);
    if (!serial_is_connected(ctx)) {
        SerialSlipError recon_err = attempt_reconnect(ctx);
        if (recon_err != SS_OK) {
            SS_LOG_ERROR("Get message failed: port not connected and reconnect failed with error code %s\n", serial_slip_decode_error(recon_err));
            return recon_err;
        }
    }
    // Używamy mutexu i zmiennej warunkowej, aby czekać na pojawienie się wiadomości.
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += timeout / 1000;
    ts.tv_nsec += (timeout % 1000) * 1000000;
    if (ts.tv_nsec >= 1000000000) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000;
    }

    pthread_mutex_lock(&ctx->queueLock);
    while (!ctx->msgHead) {
        int ret = pthread_cond_timedwait(&ctx->msgCond, &ctx->queueLock, &ts);
        if (ret == ETIMEDOUT) {
            pthread_mutex_unlock(&ctx->queueLock);
            SS_LOG_ERROR("Wait timeout\n");
            return SS_ERR_GET_MSG_TIMEOUT;
        }
    }
    // Pobieramy wiadomość, zakładając, że kolejka jest już zablokowana
    uint8_t *msgData;
    size_t len;
    if (!pop_message_locked(ctx, &msgData, &len)) {
        pthread_mutex_unlock(&ctx->queueLock);
        SS_LOG_ERROR("No message in queue despite condition signal\n");
        return SS_ERR_DECODE;
    }
    pthread_mutex_unlock(&ctx->queueLock);
    if (len > bufferSize) {
        SS_LOG_ERROR("Warning: Truncating message from %zu to %zu bytes\n", len, bufferSize);
        len = bufferSize;
    }
    memcpy(buffer, msgData, len);
    *msgLen = len;
    free(msgData);
    SS_LOG_DEBUG("Message copied successfully\n");
    return SS_OK;
}

//--------------------------------------------------------------------------
// Funkcja write and get response: wysyła dane i oczekuje na odpowiedź,
// z automatycznym ponownym wysłaniem przy błędach. Próbujemy do max_retries razy.
SerialSlipError serial_slip_write_and_get_response(SerialSlip *ctx, const uint8_t *data, size_t len, uint8_t *responseBuffer, size_t responseBufferSize, size_t *responseLen, unsigned long timeout, int max_retries)
{
    SerialSlipError err = SS_OK;
    int retry = 0;
    bool iteration_success = false;
    while ((retry < max_retries) && (!iteration_success)) {
        err = serial_slip_write(ctx, data, len);
        if (err != SS_OK) {
            SS_LOG_ERROR("Write failure in iteration, retry %d, error code: %s\n", retry, serial_slip_decode_error(err));
            retry++;
            usleep(timeout * 1000);
            continue;
        }
        err = serial_slip_get_message(ctx, responseBuffer, responseBufferSize, responseLen, timeout);
        if (err != SS_OK) {
            SS_LOG_ERROR("Receive failure in iteration, retry %d, error code: %s\n", retry, serial_slip_decode_error(err));
            retry++;
            usleep(timeout * 1000);
            continue;
        }
        iteration_success = true;
    }
    if (!iteration_success) {
        SS_LOG_ERROR("Write and get response failed after %d retries\n", max_retries);
        return err;
    }
    return SS_OK;
}

const char *serial_slip_decode_error(SerialSlipError err)
{
    return SerialSlipErrors[err];
}
