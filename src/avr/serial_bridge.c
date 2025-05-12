//
// Created by Hans-Albert Maritz on 23/03/25.
//
#include <avr/interrupt.h> // USART_RX_vect
#include "command.h" // DECL_CONSTANT_STR

#include "board/serial_bridge.h"
#include <stdint.h>
#include "generic/serial_bridge_irq.h"

// TODO: DECL_CONSTANT_STR("RESERVE_PINS_serial", "PE0,PE1");


// Define a structure that holds all the necessary pointers and bit masks for each USART.
typedef struct {
    uint8_t usart_number; // number identifier of the USART
    uint32_t baud;
    uint8_t enable_u2x;
    volatile uint8_t *ucsra;
    volatile uint8_t *ucsrb;
    volatile uint8_t *ucsrc;
    volatile uint16_t *ubrr;
    volatile uint8_t *udr;
    uint8_t u2x_bit; // Bit to set in UCSRA for double speed mode.
    uint8_t rxen_bit; // Receiver enable bit in UCSRB.
    uint8_t txen_bit; // Transmitter enable bit in UCSRB.
    uint8_t rxcie_bit; // RX complete interrupt enable bit in UCSRB.
    uint8_t udrie_bit; // UDR empty interrupt enable bit in UCSRB.
    uint8_t ucsz1_bit; // Character size bit 1 in UCSRC.
    uint8_t ucsz0_bit; // Character size bit 0 in UCSRC.
} usart_config_t;

#define USART_CONFIG(n) {                                 \
.usart_number = n,                                         \
.ucsra        = &UCSR##n##A,                              \
.ucsrb        = &UCSR##n##B,                              \
.ucsrc        = &UCSR##n##C,                              \
.ubrr         = &UBRR##n,                                 \
.udr          = &UDR##n,                                  \
.u2x_bit      = U2X##n,                                   \
.rxen_bit     = RXEN##n,                                  \
.txen_bit     = TXEN##n,                                  \
.rxcie_bit    = RXCIE##n,                                 \
.udrie_bit    = UDRIE##n,                                 \
.ucsz1_bit    = UCSZ##n##1,                               \
.ucsz0_bit    = UCSZ##n##0                                \
}

static usart_config_t serial_bridge_usarts_config[] = {
#if CONFIG_ENABLE_SERIAL_BRIDGE_USART0
    USART_CONFIG(0),
#endif
#if CONFIG_ENABLE_SERIAL_BRIDGE_USART1
    USART_CONFIG(1),
#endif
#ifdef CONFIG_ENABLE_SERIAL_BRIDGE_USART2
    USART_CONFIG(2),
#endif
#if CONFIG_ENABLE_SERIAL_BRIDGE_USART3
    USART_CONFIG(3),
#endif
};

usart_config_t *serial_bridge_get_usart_config(const uint8_t usart_number) {
    const uint8_t count = sizeof(serial_bridge_usarts_config) / sizeof(serial_bridge_usarts_config[0]);
    for (uint8_t i = 0; i < (count); i++) {
        if (serial_bridge_usarts_config[i].usart_number == usart_number) {
            return &serial_bridge_usarts_config[i];
        }
    }
    return NULL;
}

int8_t serial_bridge_get_buffer_offset_from_usart_number(const uint8_t usart_number) {
    switch (usart_number) {
        case 0: return USART0_OFFSET;
        case 1: return USART1_OFFSET;
        case 2: return USART2_OFFSET;
        case 3: return USART3_OFFSET;
        default: return -1;
    }
}


void serial_bridge_enable_tx_irq(const uint8_t usart_number) {
    usart_config_t *cfg = serial_bridge_get_usart_config(usart_number);
    if (cfg == NULL) {
        return;
    }
    uint8_t was_enabled = *(cfg->ucsrb) & (1 << cfg->udrie_bit);
    uint8_t tx_enabled = *(cfg->ucsrb) & (1 << cfg->txen_bit);
    if(was_enabled > 0 && tx_enabled > 0) {
       output("Was enabled, just gonna leave it. USART: %c", usart_number);
       return;
    }
//    output("Enabling TX interrupt for USART: %c", usart_number);
    *(cfg->ucsrb) |= (1 << cfg->udrie_bit) | (1 << cfg->txen_bit);
}

int8_t serial_bridge_configure(const uint8_t usart_number, const uint32_t baud, const uint8_t enable_u2x) {
    usart_config_t *cfg = serial_bridge_get_usart_config(usart_number);
    if (cfg == NULL) {
        return -1;
    }
    cfg->baud = baud;
    cfg->enable_u2x = enable_u2x;

    // Configure double speed mode based on configuration.
    *(cfg->ucsra) = cfg->enable_u2x ? (1 << cfg->u2x_bit) : 0;

    // Compute the clock multiplier: 8 if double speed is enabled, otherwise 16.
    const uint32_t cm = cfg->enable_u2x ? 8 : 16;

    // Set the baud rate using the divisor calculation.
    *(cfg->ubrr) = (uint16_t) (DIV_ROUND_CLOSEST(CONFIG_CLOCK_FREQ, cm * cfg->baud) - 1UL);

    // Configure the frame format: 8 data bits (no parity, 1 stop bit assumed).
    *(cfg->ucsrc) = (1 << cfg->ucsz1_bit) | (1 << cfg->ucsz0_bit);

    // Enable the receiver its interrupt.
    *(cfg->ucsrb) = (1 << cfg->rxen_bit) | (1 << cfg->rxcie_bit);

    output("Serial bridge configured %c, baud: %u", cfg->usart_number, cfg->baud);
    return 1;
}


#define DEFINE_USART_ISR(num) \
ISR(USART##num##_RX_vect) { \
    serial_bridge_rx_byte(UDR##num, USART##num##_OFFSET ); \
} \
ISR(USART##num##_UDRE_vect) { \
    uint8_t data; \
    int no_data_available = serial_bridge_get_tx_byte(&data, USART##num##_OFFSET); \
    if (no_data_available == 1) { \
        /* Disable Transmission interrupts*/\
        UCSR##num##B &= ~(1 << UDRIE##num); \
    } else { \
        UDR##num = data; \
    } \
}


// Conditionally instantiate the ISR macros for each enabled USART.
#if CONFIG_ENABLE_SERIAL_BRIDGE_USART0
DEFINE_USART_ISR(0)
#endif

#if CONFIG_ENABLE_SERIAL_BRIDGE_USART1
DEFINE_USART_ISR(1)
#endif

#if CONFIG_ENABLE_SERIAL_BRIDGE_USART2
DEFINE_USART_ISR(2)
#endif

#if CONFIG_ENABLE_SERIAL_BRIDGE_USART3
DEFINE_USART_ISR(3)
#endif
