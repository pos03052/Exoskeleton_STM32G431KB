#include "serial.h"
#include "usart.h"

void serial_write(SerialHandler *h, uint16_t _size)
{
	while(__HAL_UART_GET_FLAG(h->huart, UART_FLAG_TC) != 1) {}
	HAL_UART_Transmit_DMA(h->huart, (uint8_t *)h->tx_buffer, _size);
}

int16_t serial_read(SerialHandler *h)
{
	int16_t ret = -1;
	int16_t index = 0;

	DMA_Channel_TypeDef *hdma = h->hdma_rx->Instance;
	h->rx_buffer_head = UART_RX_BUFF_SIZE - hdma->CNDTR;
	
	if (h->rx_buffer_head != h->rx_buffer_tail) {
		index = h->rx_buffer_tail;
		ret = h->rx_buffer[index];
		h->rx_buffer_tail = (h->rx_buffer_tail + 1) % UART_RX_BUFF_SIZE;
	}
	
	return ret;
}

uint32_t serial_available(SerialHandler *h)
{
	uint32_t _length = 0;
	
	DMA_Channel_TypeDef *hdma = h->hdma_rx->Instance;
	h->rx_buffer_head = UART_RX_BUFF_SIZE - hdma->CNDTR;
	
	_length = (UART_RX_BUFF_SIZE + h->rx_buffer_head - h->rx_buffer_tail) % UART_RX_BUFF_SIZE;
	
	return _length;
}

void serial_print(SerialHandler *h, const char* _str)
{
  strcpy(h->tx_buffer, _str);
  serial_write(h, strlen(_str));
}

void serial_flush(SerialHandler *h, uint32_t timeout)
{
  //	uint32_t starttime = HAL_GetTick();
  //	while ((serial_read(h) != -1) || (HAL_GetTick() - starttime < timeout));
  h->rx_buffer_head = h->rx_buffer_tail;
}
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

void init_vcp(SerialHandler *h)
{
  h->huart = &huart2;
  h->hdma_rx = &hdma_usart2_rx;
  h->hdma_tx = &hdma_usart2_tx;
  h->rx_buffer = (char *)uart2_rx_buffer;
  h->tx_buffer = (char *)uart2_tx_buffer;	
}

__weak void parse_vcp(SerialHandler *h)
{
  
}

void run_vcp(SerialHandler *h)
{
  while (serial_available(h)) {
	h->parsing.str[h->parsing.idx] = serial_read(h);
	//		if ((h->parsing.str[h->parsing.idx] == '\r') || (h->parsing.idx >= PARSING_STR_BUFF_SIZE)) {
	if ((h->parsing.str[h->parsing.idx] == ';') || (h->parsing.idx >= PARSING_STR_BUFF_SIZE)) {
	  h->parsing.flag = 1;
	  break;
	}
	h->parsing.idx++;
  }
  
  if (h->parsing.flag) {
	h->parsing.cnt = 0;
	h->parsing.toks[h->parsing.cnt] = strtok(h->parsing.str, h->parsing.sep);	// vcp.parsing.sep = ",\r"
	while (h->parsing.toks[h->parsing.cnt] != NULL) {
	  h->parsing.toks[++h->parsing.cnt] = strtok(NULL, h->parsing.sep);
	}
	
	parse_vcp(h);
	
	h->parsing.idx = 0;
	h->parsing.flag = 0;
	memset(h->parsing.str, 0, PARSING_STR_BUFF_SIZE); // 주의! toks도 지워진다.
  }
}

SerialHandler vcp = {
  0, 0, 0, 0, 0, 0, 0,
  PARSING_DEFAULTS,
  (void (*)(uint32_t))init_vcp,
  (void (*)(uint32_t))run_vcp,
};

