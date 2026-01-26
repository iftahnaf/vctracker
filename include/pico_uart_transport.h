#ifndef PICO_UART_TRANSPORT_H
#define PICO_UART_TRANSPORT_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

struct uxrCustomTransport;

bool pico_serial_transport_open(struct uxrCustomTransport * transport);
bool pico_serial_transport_close(struct uxrCustomTransport * transport);
size_t pico_serial_transport_write(struct uxrCustomTransport * transport, const uint8_t *buf, size_t len, uint8_t *errcode);
size_t pico_serial_transport_read(struct uxrCustomTransport * transport, uint8_t *buf, size_t len, int timeout, uint8_t *errcode);

#ifdef __cplusplus
}
#endif

#endif // PICO_UART_TRANSPORT_H
