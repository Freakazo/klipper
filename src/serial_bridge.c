//
// Created by freakazo on 23/03/25.
//


#include <string.h> // memcpy
#include <stdio.h> // sprintf
#include "autoconf.h" // CONFIG_MACH_AVR
#include "board/gpio.h" // gpio_out_write
#include "board/irq.h" // irq_poll
#include "board/serial_bridge.h" // serial_bridge_configure
#include "board/misc.h" // timer_read_time
#include "board/io.h" // readb
#include "generic/serial_bridge_irq.h" // console2_sendf
#include "basecmd.h" // oid_alloc
#include "command.h" // DECL_COMMAND
#include "sched.h" // sched_shutdown

struct serial_bridge {
    struct timer timer;
    uint8_t usart_number;
    uint32_t baud;
    uint8_t u2x;
    uint32_t rest_time;
};

static struct task_wake serial_bridge_wake;

static uint_fast8_t serial_bridge_event(struct timer *timer) {
    struct serial_bridge *bridge = container_of(
        timer, struct serial_bridge, timer);

    sched_wake_task(&serial_bridge_wake);

    bridge->timer.waketime += bridge->rest_time;

    return SF_RESCHEDULE;
}


void command_config_serial_bridge(uint32_t *args) {
    struct serial_bridge *bridge = oid_alloc(
        args[0], command_config_serial_bridge, sizeof(*bridge));
    bridge->timer.func = serial_bridge_event;
    bridge->timer.waketime = timer_read_time() + args[2];
    bridge->rest_time = args[2];
    bridge->usart_number = args[3];
    bridge->baud = args[4];
    bridge->u2x = args[5];
    serial_bridge_configure(bridge->usart_number, bridge->baud, bridge->u2x);
    sched_add_timer(&bridge->timer);
}


DECL_COMMAND(command_config_serial_bridge,
             "command_config_serial_bridge oid=%c clock=%u"
             " rest_ticks=%u usart=%c baud=%u u2x=%c");


void command_serial_bridge_send(uint32_t *args) {
    struct serial_bridge *sb = oid_lookup(args[0], command_config_serial_bridge);
    uint8_t data_len = args[1];
    uint8_t *data = command_decode_ptr(args[2]);
    serial_bridge_send(data, data_len, sb->usart_number);
}

DECL_COMMAND(command_serial_bridge_send, "serial_bridge_send oid=%c data=%*s");


void serial_bridge_task(void) {
    if (!sched_check_wake(&serial_bridge_wake)) {
        return;
    }

    uint8_t buf[MESSAGE_MAX];
    uint8_t oid;
    struct serial_bridge *sb;

    foreach_oid(oid, sb, command_config_serial_bridge) {
        uint8_t data_len = serial_bridge_get_data(buf, sb->usart_number);
        if (data_len > 0) {
//            char ascii_chars[192] = {};
//            char temp[20];
//            for(uint8_t i = 0; i < data_len; i++ ) {
//               sprintf(temp, "%d", buf[i]);
//               strcat(ascii_chars, temp);
//               if (i < data_len - 1) {
//                   strcat(ascii_chars, ", ");
//               }
//            }
//            output("bridge_resp: oid %c %c %s", oid, data_len, ascii_chars);
            sendf("serial_bridge_response oid=%c data=%*s", oid, data_len, buf);
        }
    }
}
DECL_TASK(serial_bridge_task);
