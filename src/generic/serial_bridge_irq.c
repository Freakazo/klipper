#include "serial_bridge_irq.h" // serial_enable_tx_irq

#include <string.h>
#include <stdio.h>

#include "io.h"
#include "sched.h"
#include "board/irq.h"  // irq_save/irq_restore
#include "board/serial_bridge.h" //SERIAL_BRIDGE_CNT
#include "command.h" // output / MESSAGE_SYNC MESSAGE_PAYLOAD_MAX

static uint8_t receive_bridge_buf[SERIAL_BRIDGE_CNT][SERIAL_BRIDGE_RX_BUFF_SIZE] = {0};
static uint8_t receive_bridge_pos[SERIAL_BRIDGE_CNT] = {0};
static uint8_t receive_bridge_read_pos[SERIAL_BRIDGE_CNT] = {0};

static uint8_t transmit_bridge_buf [SERIAL_BRIDGE_CNT][SERIAL_BRIDGE_TX_BUFF_SIZE] = {0};
static uint8_t transmit_bridge_pos[SERIAL_BRIDGE_CNT] = {0};
static uint8_t transmit_bridge_max[SERIAL_BRIDGE_CNT] = {0};


void serial_bridge_rx_byte(uint8_t data, const uint8_t buffer_offset) {
    uint_fast8_t wpos = receive_bridge_pos[buffer_offset];
    uint_fast8_t next_wpos = (wpos + 1) % SERIAL_BRIDGE_RX_BUFF_SIZE;
    uint_fast8_t rpos = receive_bridge_read_pos[buffer_offset];

    // Check if buffer is full (next write position would equal read position)
    if (next_wpos == rpos) {
        // Buffer full - ignore
        return;
    }

    receive_bridge_buf[buffer_offset][wpos] = data;
    receive_bridge_pos[buffer_offset] = next_wpos;
    if (data == MESSAGE_SYNC) {
        sched_wake_tasks();
    }
}

uint8_t serial_bridge_get_tx_byte(uint8_t *pdata, uint8_t buffer_offset) {
    if (transmit_bridge_pos[buffer_offset] >= transmit_bridge_max[buffer_offset]) {
//        output("No data to tx");
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
//    output("tpos<%c> tmax<%c> size<%c>", tpos, tmax, size);
//    memcpy(&transmit_bridge_buf[buffer_offset][tpos], data, size); Did I accident
//    writeb(&transmit_bridge_max[buffer_offset], tpos + size);

    if (tpos >= tmax) {
        // Everything that can be sent has been sent, lets move back to the beginning of the buffer.
        tpos = tmax = 0;
        writeb(&transmit_bridge_max[buffer_offset], 0);
        writeb(&transmit_bridge_pos[buffer_offset], 0);
    }
    if (tmax + size - tpos > SERIAL_BRIDGE_TX_BUFF_SIZE) {
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
    uint8_t new_tmax = readb(&transmit_bridge_max[buffer_offset]);
    char ascii_chars[192] = {};
    char temp[20];
    for(uint8_t i = 0; i < new_tmax; i++ ) {
       sprintf(temp, "%d", transmit_bridge_buf[buffer_offset][i]);
       strcat(ascii_chars, temp);
       if (i < size - 1) {
           strcat(ascii_chars, ", ");
       }
    }
//    output("end of thing - tpos<%c> tmax<%c> size<%c> contents: %s", tpos, tmax, size, ascii_chars);
    serial_bridge_enable_tx_irq(usart_number);
}


uint8_t serial_bridge_get_data(uint8_t *data, uint8_t usart_number) {
    const int8_t buffer_offset = serial_bridge_get_buffer_offset_from_usart_number(usart_number);
    if (buffer_offset < 0) {
        return -1;
    }
    for (;;) {
        uint_fast8_t wpos = readb(&receive_bridge_pos[buffer_offset]);
        uint_fast8_t rpos = readb(&receive_bridge_read_pos[buffer_offset]);


        // No new data to read
        if (wpos == rpos) {
            return 0;
        }

        uint8_t *buf = receive_bridge_buf[buffer_offset];
        uint_fast8_t data_size = 0;

        // Handle ring buffer wraparound
        if (wpos > rpos) {
            // Simple case: read from rpos to wpos
            data_size = wpos - rpos;
            if (data_size > MESSAGE_PAYLOAD_MAX) {
                data_size = MESSAGE_PAYLOAD_MAX;
            }
            memcpy(data, &buf[rpos], data_size);
        } else {
            // Wraparound case: read from rpos to end, then from start to wpos
            data_size = SERIAL_BRIDGE_RX_BUFF_SIZE - rpos;
            memcpy(data, &buf[rpos], data_size);
            if (wpos > 0) {
                uint8_t max_to_read = ((wpos) < (MESSAGE_PAYLOAD_MAX - data_size) ? (wpos) : (MESSAGE_PAYLOAD_MAX - data_size));
                memcpy(data + data_size, buf, max_to_read);
                data_size += max_to_read;
            }
        }

        // Update read position with wraparound
        rpos = (rpos + data_size) % SERIAL_BRIDGE_RX_BUFF_SIZE;
        writeb(&receive_bridge_read_pos[buffer_offset], rpos);
        return data_size;
    }
}
