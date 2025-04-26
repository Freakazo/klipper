//
// Created by Hans-Albert Maritz on 23/03/25.
//

#ifndef SERIAL_BRIDGE_IRQ_H
#define SERIAL_BRIDGE_IRQ_H

#include <stdint.h> // uint32_t

#define SERIAL_BRIDGE_RX_BUFF_SIZE 192
#define SERIAL_BRIDGE_TX_BUFF_SIZE 192

// callback provided by board specific code
void serial_bridge_enable_tx_irq(uint8_t usart_index);

// serial_bridge_irq.c
void serial_bridge_rx_byte(uint_fast8_t data, uint8_t buffer_offset);
uint8_t serial_bridge_get_tx_byte(uint8_t *pdata, uint8_t buffer_offset); // returns 1 if no data available, otherwise 0.

// serial_bridge_irq.c
void serial_bridge_send(const uint8_t* data, uint_fast8_t size, uint8_t usart_number);

// serial_bridge_irq.c
uint8_t serial_bridge_get_data(uint8_t* data, uint8_t usart_number);

#endif //SERIAL_BRIDGE_IRQ_H
