#include "serial_bridge_irq.h" // serial_enable_tx_irq

#include <string.h>

#include "io.h"
#include "sched.h"
#include "./irq.h" // irq_save/irq_restore
#include "board/serial_bridge.h" //SERIAL_BRIDGE_CNT

// TODO: Are these initialised to 0?
static uint8_t receive_bridge_buf[SERIAL_BRIDGE_CNT][SERIAL_BRIDGE_RX_BUFF_SIZE],
        receive_bridge_pos[SERIAL_BRIDGE_CNT];

static uint8_t transmit_bridge_buf
        [SERIAL_BRIDGE_CNT][SERIAL_BRIDGE_TX_BUFF_SIZE],
        transmit_bridge_pos[SERIAL_BRIDGE_CNT], transmit_bridge_max[SERIAL_BRIDGE_CNT];


void serial_bridge_rx_byte(uint_fast8_t data, const uint8_t buffer_offset) {
    if (receive_bridge_pos[buffer_offset] >= SERIAL_BRIDGE_RX_BUFF_SIZE) {
        // Serial overflow - ignore
        return;
    }
    receive_bridge_buf[buffer_offset][receive_bridge_pos[buffer_offset]++] = data;
    sched_wake_tasks();
}

uint8_t serial_bridge_get_tx_byte(uint8_t *pdata, uint8_t buffer_offset) {
    if (transmit_bridge_pos[buffer_offset] >= transmit_bridge_max[buffer_offset]) {
        return 1;
    }
    *pdata = transmit_bridge_buf[buffer_offset][transmit_bridge_pos[buffer_offset]++];
    return 0;
}

void serial_bridge_send(const uint8_t *data, uint_fast8_t size, uint8_t usart_number) {
    // Get the buffer_offset so we know which buffer to write to
    const int8_t buffer_offset = serial_bridge_get_buffer_offset_from_usart_number(usart_number);
    if (buffer_offset < 0) {
        // The usart is not enabled when compiled due to config.
        return;
    }
    uint_fast8_t tpos = readb(&transmit_bridge_pos[buffer_offset]);
    uint_fast8_t tmax = readb(&transmit_bridge_max[buffer_offset]);
    if (tpos >= tmax) {
        // TODO?: Is this overflow protection? Making a ring bugger? Will we override previous data to send? Only 1 byte at a time
        tpos = tmax = 0;
        writeb(&transmit_bridge_max[buffer_offset], 0);
        writeb(&transmit_bridge_pos[buffer_offset], 0);
    }
    if (tmax + size - tpos) {
        return; // Message is too large to store in the buffer
    }
    if (tmax + size > SERIAL_BRIDGE_TX_BUFF_SIZE) {
        // Need to move the existing data in the buffer to the start of it to fit in the new message.
        writeb(&transmit_bridge_max[buffer_offset], 0);
        // Prevents transmitting contents of the buffer whilst we move it arround.
        tpos = readb(&transmit_bridge_pos[buffer_offset]);
        tmax -= tpos; // tmax is now the length of existing data in the ring buffer.
        memmove(&transmit_bridge_buf[buffer_offset][0], &transmit_bridge_buf[buffer_offset][tpos], tmax);
        writeb(&transmit_bridge_pos[buffer_offset], 0);
        writeb(&transmit_bridge_max[buffer_offset], tmax);
        serial_bridge_enable_tx_irq(usart_number); // Can
    }
    // Copy the message into the buffer
    memcpy(&transmit_bridge_buf[buffer_offset][tmax], data, size);
    // Update the data end position
    writeb(&transmit_bridge_max[buffer_offset], tmax + size);
    serial_bridge_enable_tx_irq(usart_number);
}


uint8_t serial_bridge_get_data(uint8_t *data, uint8_t usart_number) {
    const int8_t buffer_offset = serial_bridge_get_buffer_offset_from_usart_number(usart_number);
    if (buffer_offset < 0) {
        return -1;
    }

    for (;;) {
        uint_fast8_t rpos = readb(&receive_bridge_pos[buffer_offset]);
        if (!rpos)
            return 0;
        uint8_t *buf = receive_bridge_buf[buffer_offset];
        memcpy(data, buf, rpos);
        irqstatus_t flag = irq_save();

        if (rpos != readb(&receive_bridge_pos[buffer_offset])) {
            // Raced with irq handler - retry
            irq_restore(flag);
            continue;
        }
        receive_bridge_pos[buffer_offset] = 0;
        irq_restore(flag);
        return rpos;
    }
}
