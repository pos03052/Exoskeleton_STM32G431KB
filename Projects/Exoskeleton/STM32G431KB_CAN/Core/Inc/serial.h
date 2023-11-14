#ifndef __SERIAL_H
#define __SERIAL_H

#ifdef __cplusplus
extern "C" {
#endif
	
#include "main.h"

#define PARSING_STR_BUFF_SIZE	64
#define PARSING_TOK_BUFF_SIZE	16
#define PARSING_DEFAULTS		{0, 0, 0, {0,}, ",\r", {0,}}

typedef struct {
	uint8_t flag;
	uint16_t idx;
	uint16_t cnt;
	char str[PARSING_STR_BUFF_SIZE];
	char *sep;
	char *toks[PARSING_TOK_BUFF_SIZE];
} ParsingHandler;

typedef struct {
	UART_HandleTypeDef *huart;
	DMA_HandleTypeDef *hdma_rx;
	DMA_HandleTypeDef *hdma_tx;
	
	char *tx_buffer;
	char *rx_buffer;
	
	uint32_t rx_buffer_head;
	uint32_t rx_buffer_tail;

	ParsingHandler parsing;
	
	void (*init)();
	void (*run)();
} SerialHandler;

void serial_print(SerialHandler *h, const char* _str);
void serial_write(SerialHandler *h, uint16_t _size);
int16_t serial_read(SerialHandler *h);
uint32_t serial_available(SerialHandler *h);
void serial_flush(SerialHandler *h, uint32_t timeout);

void init_vcp(SerialHandler *h);
void run_vcp(SerialHandler *h);
void parse_vcp(SerialHandler *h);

extern SerialHandler vcp;

#ifdef __cplusplus
}
#endif

#endif /* __SERIAL_H */