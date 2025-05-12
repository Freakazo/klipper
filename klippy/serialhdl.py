# Serial port management for firmware communication
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, threading, os
import pty
import select
import termios
import tty

import serial

import msgproto, chelper, util

class error(Exception):
    pass

class SerialReader:
    def __init__(self, reactor, warn_prefix=""):
        self.reactor = reactor
        self.warn_prefix = warn_prefix
        # Serial port
        self.serial_dev = None
        self.msgparser = msgproto.MessageParser(warn_prefix=warn_prefix)
        # C interface
        self.ffi_main, self.ffi_lib = chelper.get_ffi()
        self.serialqueue = None
        self.default_cmd_queue = self.alloc_command_queue()
        self.stats_buf = self.ffi_main.new('char[4096]')
        # Threading
        self.lock = threading.Lock()
        self.background_thread = None
        # Message handlers
        self.handlers = {}
        self.register_response(self._handle_unknown_init, '#unknown')
        self.register_response(self.handle_output, '#output')
        # Sent message notification tracking
        self.last_notify_id = 0
        self.pending_notifications = {}

    def _bg_thread(self):
        response = self.ffi_main.new('struct pull_queue_message *')
        while 1:
            self.ffi_lib.serialqueue_pull(self.serialqueue, response)
            count = response.len
            if count < 0:
                break
            if response.notify_id:
                params = {'#sent_time': response.sent_time,
                          '#receive_time': response.receive_time}
                # logging.info('%sGot notified for id %s - %s', self.warn_prefix, response.notify_id, params)
                completion = self.pending_notifications.pop(response.notify_id)
                self.reactor.async_complete(completion, params)
                continue
            params = self.msgparser.parse(response.msg[0:count])
            params['#sent_time'] = response.sent_time
            params['#receive_time'] = response.receive_time
            # if params.get('#name') not in ['#output', 'analog_in_state', 'clock']:
            #     logging.info('%sReceived response from serial %s', self.warn_prefix, params)
            hdl = (params['#name'], params.get('oid'))
            try:
                with self.lock:
                    hdl = self.handlers.get(hdl, self.handle_default)
                    hdl(params)
            except:
                logging.exception("%sException in serial callback",
                                  self.warn_prefix)
    def _error(self, msg, *params):
        raise error(self.warn_prefix + (msg % params))
    def _get_identify_data(self, eventtime):
        logging.info('Attempting to get logging data from micro-controller..')
        # Query the "data dictionary" from the micro-controller
        identify_data = b""
        while 1:
            msg = "identify offset=%d count=%d" % (len(identify_data), 40)
            try:
                params = self.send_with_response(msg, 'identify_response')
            except error as e:
                logging.exception("%sWait for identify_response",
                                  self.warn_prefix)
                return None
            if params['offset'] == len(identify_data):
                msgdata = params['data']
                if not msgdata:
                    # Done
                    return identify_data
                identify_data += msgdata
    def _start_session(self, serial_dev, serial_fd_type=b'u', client_id=0):
        self.serial_dev = serial_dev
        self.serialqueue = self.ffi_main.gc(
            self.ffi_lib.serialqueue_alloc(serial_dev.fileno(),
                                           serial_fd_type, client_id),
            self.ffi_lib.serialqueue_free)
        self.background_thread = threading.Thread(target=self._bg_thread)
        self.background_thread.start()
        logging.info('%sStarting serial session with %s', self.warn_prefix, serial_fd_type)
        # Obtain and load the data dictionary from the firmware
        completion = self.reactor.register_callback(self._get_identify_data)
        identify_data = completion.wait(self.reactor.monotonic() + 50.)
        if identify_data is None:
            logging.info("%sTimeout on connect", self.warn_prefix)
            logging.info("%s Dump Debug:\n%s", self.warn_prefix, self.dump_debug())

            self.disconnect()
            return False
        msgparser = msgproto.MessageParser(warn_prefix=self.warn_prefix)
        msgparser.process_identify(identify_data)
        self.msgparser = msgparser
        self.register_response(self.handle_unknown, '#unknown')
        # Setup baud adjust
        if serial_fd_type == b'c':
            wire_freq = msgparser.get_constant_float('CANBUS_FREQUENCY', None)
        else:
            wire_freq = msgparser.get_constant_float('SERIAL_BAUD', None)
        if wire_freq is not None:
            self.ffi_lib.serialqueue_set_wire_frequency(self.serialqueue,
                                                        wire_freq)
        receive_window = msgparser.get_constant_int('RECEIVE_WINDOW', None)
        if receive_window is not None:
            self.ffi_lib.serialqueue_set_receive_window(
                self.serialqueue, receive_window)
        return True
    def connect_canbus(self, canbus_uuid, canbus_nodeid, canbus_iface="can0"):
        import can # XXX
        txid = canbus_nodeid * 2 + 256
        filters = [{"can_id": txid+1, "can_mask": 0x7ff, "extended": False}]
        # Prep for SET_NODEID command
        try:
            uuid = int(canbus_uuid, 16)
        except ValueError:
            uuid = -1
        if uuid < 0 or uuid > 0xffffffffffff:
            self._error("Invalid CAN uuid")
        uuid = [(uuid >> (40 - i*8)) & 0xff for i in range(6)]
        CANBUS_ID_ADMIN = 0x3f0
        CMD_SET_NODEID = 0x01
        set_id_cmd = [CMD_SET_NODEID] + uuid + [canbus_nodeid]
        set_id_msg = can.Message(arbitration_id=CANBUS_ID_ADMIN,
                                 data=set_id_cmd, is_extended_id=False)
        # Start connection attempt
        logging.info("%sStarting CAN connect", self.warn_prefix)
        start_time = self.reactor.monotonic()
        while 1:
            if self.reactor.monotonic() > start_time + 90.:
                self._error("Unable to connect")
            try:
                bus = can.interface.Bus(channel=canbus_iface,
                                        can_filters=filters,
                                        bustype='socketcan')
                bus.send(set_id_msg)
            except (can.CanError, os.error, IOError) as e:
                logging.warning("%sUnable to open CAN port: %s",
                                self.warn_prefix, e)
                self.reactor.pause(self.reactor.monotonic() + 5.)
                continue
            bus.close = bus.shutdown # XXX
            ret = self._start_session(bus, b'c', txid)
            if not ret:
                continue
            # Verify correct canbus_nodeid to canbus_uuid mapping
            try:
                params = self.send_with_response('get_canbus_id', 'canbus_id')
                got_uuid = bytearray(params['canbus_uuid'])
                if got_uuid == bytearray(uuid):
                    break
            except:
                logging.exception("%sError in canbus_uuid check",
                                  self.warn_prefix)
            logging.info("%sFailed to match canbus_uuid - retrying..",
                         self.warn_prefix)
            self.disconnect()
    def connect_pipe(self, filename):
        logging.info("%sStarting connect", self.warn_prefix)
        start_time = self.reactor.monotonic()
        while 1:
            if self.reactor.monotonic() > start_time + 90.:
                self._error("Unable to connect")
            try:
                fd = os.open(filename, os.O_RDWR | os.O_NOCTTY)
            except OSError as e:
                logging.warning("%sUnable to open port: %s",
                                self.warn_prefix, e)
                self.reactor.pause(self.reactor.monotonic() + 5.)
                continue
            serial_dev = os.fdopen(fd, 'wb+', 0)
            ret = self._start_session(serial_dev)
            if ret:
                break
    def connect_uart(self, serialport, baud, rts=True):
        # Initial connection
        logging.info("%sStarting serial connect", self.warn_prefix)
        start_time = self.reactor.monotonic()
        while 1:
            if self.reactor.monotonic() > start_time + 90.:
                self._error("Unable to connect")
            try:
                serial_dev = serial.Serial(baudrate=baud, timeout=0,
                                           exclusive=True)
                serial_dev.port = serialport
                serial_dev.rts = rts
                serial_dev.open()
            except (OSError, IOError, serial.SerialException) as e:
                logging.warning("%sUnable to open serial port: %s",
                             self.warn_prefix, e)
                self.reactor.pause(self.reactor.monotonic() + 5.)
                continue
            stk500v2_leave(serial_dev, self.reactor)
            ret = self._start_session(serial_dev)
            logging.info("%sStarted serial connection", self.warn_prefix)
            if ret:
                break
    def connect_file(self, debugoutput, dictionary, pace=False):
        self.serial_dev = debugoutput
        self.msgparser.process_identify(dictionary, decompress=False)
        self.serialqueue = self.ffi_main.gc(
            self.ffi_lib.serialqueue_alloc(self.serial_dev.fileno(), b'f', 0),
            self.ffi_lib.serialqueue_free)

    def connect_serial_bridge(self, bridge_mcu, oid):
        """Connect to a microcontroller via another microcontroller's serial port

        This method sets up a connection to a target microcontroller through
        another microcontroller (bridge_mcu) using its serial port.

        Args:
            bridge_mcu: The bridge microcontroller object
            usart: The USART port number on the bridge MCU (default: 0)
            baud: The baud rate for the serial connection (default: 250000)
            u2x: Double transmission speed flag (default: 0)
        """
        self.bridge_mcu = bridge_mcu
        self.bridge_oid = oid
        # Create a SerialBridgeDevice for communication
        self.serial_dev = SerialBridgeDevice(self.bridge_mcu, self.bridge_oid, self.get_default_command_queue())

        # Start a new session with the serial bridge device
        logging.info("%sStarting serial bridge session", self.warn_prefix)
        ret = self._start_session(self.serial_dev)
        if not ret:
            self._error("Unable to start session with serial bridge")
        logging.info('Started Serial Bridge Session')

    def set_clock_est(self, freq, conv_time, conv_clock, last_clock):
        self.ffi_lib.serialqueue_set_clock_est(
            self.serialqueue, freq, conv_time, conv_clock, last_clock)
    def disconnect(self):
        if self.serialqueue is not None:
            self.ffi_lib.serialqueue_exit(self.serialqueue)
            if self.background_thread is not None:
                self.background_thread.join()
            self.background_thread = self.serialqueue = None
        if self.serial_dev is not None:
            self.serial_dev.close()
            self.serial_dev = None
        # Clean up bridge MCU registration if it exists
        if hasattr(self, 'bridge_mcu') and hasattr(self, 'bridge_oid'):
            self.bridge_mcu.register_response(None, 'serial_bridge_response', self.bridge_oid)
            delattr(self, 'bridge_mcu')
            delattr(self, 'bridge_oid')
        for pn in self.pending_notifications.values():
            pn.complete(None)
        self.pending_notifications.clear()
    def stats(self, eventtime):
        if self.serialqueue is None:
            return ""
        self.ffi_lib.serialqueue_get_stats(self.serialqueue,
                                           self.stats_buf, len(self.stats_buf))
        return str(self.ffi_main.string(self.stats_buf).decode())
    def get_reactor(self):
        return self.reactor
    def get_msgparser(self):
        return self.msgparser
    def get_serialqueue(self):
        return self.serialqueue
    def get_default_command_queue(self):
        return self.default_cmd_queue
    # Serial response callbacks
    def register_response(self, callback, name, oid=None):
        with self.lock:
            if callback is None:
                del self.handlers[name, oid]
            else:
                self.handlers[name, oid] = callback
    # Command sending
    def raw_send(self, cmd, minclock, reqclock, cmd_queue):
        # logging.info('%sraw_send %s', self.warn_prefix, cmd)
        # if hasattr(self, 'bridge_mcu') and hasattr(self, 'bridge_oid'):
        #     encoded_cmd = self._encode_bridge_cmd(cmd)
        #     self._serial_bridge_send_cmd.send([self.bridge_oid, encoded_cmd])
        #     return
        self.ffi_lib.serialqueue_send(self.serialqueue, cmd_queue,
                                      cmd, len(cmd), minclock, reqclock, 0)
    def raw_send_wait_ack(self, cmd, minclock, reqclock, cmd_queue):
        # if hasattr(self, 'bridge_mcu') and hasattr(self, 'bridge_oid'):
        #     encoded_cmd = self._encode_bridge_cmd(cmd)
        #     logging.info("%sbridged raw_send_wait_ack - oid %s, cmd %s encoded[%s]", self.warn_prefix, self.bridge_oid, cmd, encoded_cmd)
        #     params = self._serial_bridge_send_cmd.send_wait_ack([self.bridge_oid, encoded_cmd], minclock, reqclock)
        #     logging.info("%s - bridge got send_wait_ack response, %s", self.warn_prefix, params)
        #     if params is None:
        #         self._error("Serial connection closed")
        #     return params

        self.last_notify_id += 1
        nid = self.last_notify_id
        # logging.info('%s raw_send_wait_ack %s nid[%s]', self.warn_prefix, cmd, nid)
        completion = self.reactor.completion()
        self.pending_notifications[nid] = completion
        self.ffi_lib.serialqueue_send(self.serialqueue, cmd_queue,
                                      cmd, len(cmd), minclock, reqclock, nid)
        params = completion.wait()
        if params is None:
            self._error("Serial connection closed")
        return params
    def send(self, msg, minclock=0, reqclock=0):
        cmd = self.msgparser.create_command(msg)
        self.raw_send(cmd, minclock, reqclock, self.default_cmd_queue)
    def send_with_response(self, msg, response):
        # if hasattr(self, 'bridge_mcu'):
        #     # Maybe we have to send the bridge command.
        #     # Bridge responses should be parse.
        #     # Bridge responses should populate the queue and messages be pulled from it.
        #     cmd = self.msgparser.create_command(msg)
        #     encoded_cmd = self._encode_bridge_cmd(cmd)
        #     bridge_send_cmd = "serial_bridge_send oid=%d data=%s" % (self.bridge_oid, encoded_cmd)
        #     logging.info('%sBridged send_with_response cmd[%s] msg[%s enc<%s>] response: %s',self.warn_prefix, bridge_send_cmd, msg, encoded_cmd, response)
        #     return self.bridge_mcu._serial.send_with_response(bridge_send_cmd, 'serial_bridge_response')
        cmd = self.msgparser.create_command(msg)
        # logging.info('%ssend_with_response cmd[%s] msg[%s] for response: %s',self.warn_prefix, cmd, msg, response)
        src = SerialRetryCommand(self, response)
        return src.get_response([cmd], self.default_cmd_queue)

    # def _encode_bridge_cmd(self, cmd):
    #     raw_message = self.get_msgparser().encode_msgblock(0, cmd)
    #     logging.info("The raw message is: %s", raw_message)
    #     flat_list = [item for sublist in raw_message for item in (sublist if isinstance(sublist, list) else [sublist])]
    #     encoded_cmd = bytearray(flat_list)
    #     logging.info("The encoded message is: %s", encoded_cmd)
    #     return bytes(encoded_cmd).hex()
    def alloc_command_queue(self):
        return self.ffi_main.gc(self.ffi_lib.serialqueue_alloc_commandqueue(),
                                self.ffi_lib.serialqueue_free_commandqueue)
    # Dumping debug lists
    def dump_debug(self):
        out = []
        out.append("Dumping serial stats: %s" % (
            self.stats(self.reactor.monotonic()),))
        sdata = self.ffi_main.new('struct pull_queue_message[1024]')
        rdata = self.ffi_main.new('struct pull_queue_message[1024]')
        scount = self.ffi_lib.serialqueue_extract_old(self.serialqueue, 1,
                                                      sdata, len(sdata))
        rcount = self.ffi_lib.serialqueue_extract_old(self.serialqueue, 0,
                                                      rdata, len(rdata))
        out.append("Dumping send queue %d messages" % (scount,))
        for i in range(scount):
            msg = sdata[i]
            cmds = self.msgparser.dump(msg.msg[0:msg.len])
            out.append("Sent %d %f %f %d: %s" % (
                i, msg.receive_time, msg.sent_time, msg.len, ', '.join(cmds)))
        out.append("Dumping receive queue %d messages" % (rcount,))
        for i in range(rcount):
            msg = rdata[i]
            cmds = self.msgparser.dump(msg.msg[0:msg.len])
            out.append("Receive: %d %f %f %d: %s" % (
                i, msg.receive_time, msg.sent_time, msg.len, ', '.join(cmds)))
        return '\n'.join(out)
    # Default message handlers
    def _handle_unknown_init(self, params):
        logging.debug("%sUnknown message %d (len %d) while identifying",
                      self.warn_prefix, params['#msgid'], len(params['#msg']))
    def handle_unknown(self, params):
        logging.warning("%sUnknown message type %d: %s",
                     self.warn_prefix, params['#msgid'], repr(params['#msg']))
    def handle_output(self, params):
        logging.info("%s%s: %s", self.warn_prefix,
                     params['#name'], params['#msg'])
    def handle_default(self, params):
        logging.warning("%sgot %s", self.warn_prefix, params)

# Class to send a query command and return the received response
class SerialRetryCommand:
    def __init__(self, serial, name, oid=None):
        self.serial = serial
        self.name = name
        self.oid = oid
        self.last_params = None
        self.serial.register_response(self.handle_callback, name, oid)
    def handle_callback(self, params):
        self.last_params = params
    def get_response(self, cmds, cmd_queue, minclock=0, reqclock=0):
        retries = 5
        retry_delay = .010
        while 1:
            # logging.info('Serial Retry sending: %', cmds)
            for cmd in cmds[:-1]:
                self.serial.raw_send(cmd, minclock, reqclock, cmd_queue)
            self.serial.raw_send_wait_ack(cmds[-1], minclock, reqclock,
                                          cmd_queue)
            params = self.last_params
            if params is not None:
                self.serial.register_response(None, self.name, self.oid)
                return params
            if retries <= 0:
                self.serial.register_response(None, self.name, self.oid)
                raise error("Unable to obtain '%s' response" % (self.name,))
            reactor = self.serial.reactor
            reactor.pause(reactor.monotonic() + retry_delay)
            retries -= 1
            retry_delay *= 2.

# Attempt to place an AVR stk500v2 style programmer into normal mode
def stk500v2_leave(ser, reactor):
    logging.debug("Starting stk500v2 leave programmer sequence")
    util.clear_hupcl(ser.fileno())
    origbaud = ser.baudrate
    # Request a dummy speed first as this seems to help reset the port
    ser.baudrate = 2400
    ser.read(1)
    # Send stk500v2 leave programmer sequence
    ser.baudrate = 115200
    reactor.pause(reactor.monotonic() + 0.100)
    ser.read(4096)
    ser.write(b'\x1b\x01\x00\x01\x0e\x11\x04')
    reactor.pause(reactor.monotonic() + 0.050)
    res = ser.read(4096)
    logging.debug("Got %s from stk500v2", repr(res))
    ser.baudrate = origbaud

def cheetah_reset(serialport, reactor):
    # Fysetc Cheetah v1.2 boards have a weird stateful circuitry for
    # configuring the bootloader. This sequence takes care of disabling it for
    # sure.
    # Open the serial port with RTS asserted
    ser = serial.Serial(baudrate=2400, timeout=0, exclusive=True)
    ser.port = serialport
    ser.rts = True
    ser.open()
    ser.read(1)
    reactor.pause(reactor.monotonic() + 0.100)
    # Toggle DTR
    ser.dtr = True
    reactor.pause(reactor.monotonic() + 0.100)
    ser.dtr = False
    # Deassert RTS
    reactor.pause(reactor.monotonic() + 0.100)
    ser.rts = False
    reactor.pause(reactor.monotonic() + 0.100)
    # Toggle DTR again
    ser.dtr = True
    reactor.pause(reactor.monotonic() + 0.100)
    ser.dtr = False
    reactor.pause(reactor.monotonic() + 0.100)
    ser.close()

# Attempt an arduino style reset on a serial port
def arduino_reset(serialport, reactor):
    # First try opening the port at a different baud
    ser = serial.Serial(serialport, 2400, timeout=0, exclusive=True)
    ser.read(1)
    reactor.pause(reactor.monotonic() + 0.100)
    # Then toggle DTR
    ser.dtr = True
    reactor.pause(reactor.monotonic() + 0.100)
    ser.dtr = False
    reactor.pause(reactor.monotonic() + 0.100)
    ser.close()


# Serial Bridge Device class to handle communication through a bridge MCU
class SerialBridgeDevice:
    def __init__(self, bridge_mcu, oid, cmd_queue):
        self.bridge_mcu = bridge_mcu
        self.oid = oid
        self.read_lock = threading.Lock()
        self.write_lock = threading.Lock()
        # Create a pipe for serialqueue to read from
        self.master_fd, self.slave_fd = pty.openpty()
        new_settings = termios.tcgetattr(self.master_fd)  # Does this to avoid modifying a reference that also modifies old_settings
        new_settings[3] = new_settings[3] & ~termios.ECHO
        termios.tcsetattr(self.master_fd, termios.TCSADRAIN, new_settings)
        tty.setraw(self.master_fd, termios.TCSADRAIN)

        self.send_cmd = bridge_mcu.lookup_command("serial_bridge_send oid=%c data=%*s", cmd_queue)
        bridge_mcu.register_response(self._handle_serial_bridge_response, 'serial_bridge_response', self.oid)

        self.master_thread = threading.Thread(target=self._listener)
        self.master_thread.start()

    def _listener(self):
        MAX_MSG_SIZE = 64 - 8
        while 1:
            with self.write_lock:
                r, w, e, = select.select([self.master_fd], [], [], 0)
                if not r:
                    continue
                data = os.read(self.master_fd, MAX_MSG_SIZE)
                if not data:
                    return
                # logging.info('mmu3 _listener, %s', list(bytearray(data)))
                self.send_cmd.send([self.oid, data])

    def _handle_serial_bridge_response(self, params):
        data = params['data']
        # logging.info('_handle_serial_bridge_response, %s', list(bytearray(data)))
        with self.write_lock:
            os.write(self.master_fd, bytearray(data))

    def fileno(self):
        # Return the file descriptor for serialqueue to read from
        return self.slave_fd


    def close(self):
        logging.info('Closing the pipe')
        os.close(self.slave_fd)
        os.close(self.master_fd)

#
#
#
#
#
#