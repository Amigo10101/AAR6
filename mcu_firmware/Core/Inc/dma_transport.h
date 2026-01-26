#ifndef DMA_TRANSPORT_H
#define DMA_TRANSPORT_H

#include <uxr/client/transport.h>
#include "main.h" // For UART handle

bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

#endif // DMA_TRANSPORT_H
