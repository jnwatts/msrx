""" WIP library for the MSR605X

2020-09 Josh Watts: add read/write/flush API

From README.md:

    # WIP MSR605X magstripe reader/writer library

    This is an in progress library for the MSR605X magstripe reader/writer.  The MSR605X appears as a USB HID device and uses a protocol that appears to be a small wrapper around the older serial protocol used by other MSR
    * devices.

    # Protocol Details

    The MSR605X uses 64 byte USB HID packets to encapsulate what appears to be the MSR605's serial protocol.  
    The MSR605's serial protocol is documented in section 6 of the [MSR605 Programmer's Manual](https://usermanual.wiki/Pdf/MSR60520Programmers20Manual.325315846/help).

    Messages to be sent over USB are split into chunks with a maximum size of 63 bytes. A 64 byte USB HID packet is then constructed from a 1 byte header, a chunk of the message, and sometimes some extra bytes to make the packet exactly 64 bytes regardless of the size of the chunk. The 1 byte header is made up of a single bit indicating if this packet is the first in the sequence of packets encapsulating a particular message, another single bit that indicates if this is the last packet in a sequence, and a 6 bit unsigned integer representing the length of the payload in the current packet.  For example, a header byte of 0b11000010 (0xC2) has the first packet in sequence bit (0b10000000) set, the last packet in sequence bit (0b01000000) set, and has a payload length of 0b00000010 (2).

    # Encapsulation Examples

    Note: while in the examples, I fill in the bytes necessary to make each packet 64 bytes long with zeroes, the actual reader and it's companion software does not and it looks like it justs sends whatever was last in memory where it's reading from (this tripped me up a bunch early on since I was trying to make sense of the whole packet).

    ## Single Packet Example

    Message: "string"

    Packet 1: 0xC6737472696E67000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000

    Packet 1 header: 0b11000110 (0xC6): start of sequence, end of sequence, this packet's payload length is 6 (0b000110)

    Packet 1 payload: 0x737472696E67 ("string")

    ## Multiple Packet Example

    Message: "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA"

    Packet 1: 0xBF414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141

    Packet 1 header: 0b10111111 (0xBF): start of sequence, this packet's payload length is 63 (0b00111111)

    Packet 2: 0x3F414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141414141

    Packet 2 header: 0b00111111 (0x3F): this packet's payload length is 63 (0b00111111)

    Packet 3: 0x4F414141414141414141414141414141000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000

    Packet 3 header: 0b01001111 (0x4F): end of sequence, this packet's payload length is 15 (0b001111)

    # Requirements

    - python 3
    - pyusb from pip or another package manager (the package is often `python-pyusb` or similar)
    - suitable USB backend for pyusb, see the [pyusb tutorial](https://github.com/pyusb/pyusb/blob/master/docs/tutorial.rst) for more information on backends

    # Debugging Setup
    - MSR605X physically attached to linux host
      - passed through to windows VM
    - usbmon kernel module on host intercepts the USB packets
      - Wireshark records intercepted USB packets
    - Windows VM with MSR605X gui
"""

import usb
import usb.core

SEQUENCE_START_BIT = 0b10000000
SEQUENCE_END_BIT = 0b01000000
SEQUENCE_LENGTH_BITS = 0b00111111
ESC = b"\x1b"

class MSR605X:
    """ Represents a MSR605X device

    There are three levels of abstraction that this class can be used at:
    - raw 64 byte hid packets: _send_packet and _recv_packet
    - plain MSR605 serial protocol messages: send_message and recv_message
    - higher level functions: reset, ... (more to be added)

    """
    def __init__(self, **kwargs):
        if "idVendor" not in kwargs:
            kwargs["idVendor"] = 0x0801
            kwargs["idProduct"] = 0x0003
        self.dev = usb.core.find(**kwargs)
        self.hid_endpoint = None
        self.buffer = b''
    def connect(self):
        """ Establish a connection to the MSR605X """
        dev = self.dev
        if dev.is_kernel_driver_active(0):
            dev.detach_kernel_driver(0)
        config = dev.get_active_configuration()
        if config is None or config.bConfigurationValue != 1:
            dev.set_configuration()
            config = dev.get_active_configuration()
        interface = config.interfaces()[0]
        self.hid_endpoint = interface.endpoints()[0]
    # def disconnect(self):
        # dev = self.dev
        # dev.detach_kernel_driver(0)
    def _make_header(self, start_of_sequence: bool, end_of_sequence: bool, length: int):
        if length < 0 or length > 63:
            raise ValueError("Length must be a non-negative number no more than 63")
        header = length
        if start_of_sequence:
            header |= SEQUENCE_START_BIT
        if end_of_sequence:
            header |= SEQUENCE_END_BIT
        return bytes([header])
    def _encapsulate_message(self, message):
        idx = 0
        while idx < len(message):
            payload = message[idx:idx+63]
            header = self._make_header(idx == 0, len(message) - idx < 64, len(payload))
            padding = b"\0" * (63 - len(payload))
            yield header + payload + padding
            idx += 63
    def _send_packet(self, packet):
        self.dev.ctrl_transfer(0x21, 9, wValue=0x0300, wIndex=0, data_or_wLength=packet)
    def _recv_packet(self, **kwargs):
        try:
            return bytes(self.hid_endpoint.read(64, **kwargs))
        except usb.core.USBError as error:
            if error.errno == 110:
                return None
            raise error
    def send_message(self, message):
        """ Send a message to the MSR605X """
        for packet in self._encapsulate_message(message):
            self._send_packet(packet)
    def recv_message(self, timeout=0):
        """ Receive message from the MSR605X """
        message = b""
        while True:
            packet = self._recv_packet(timeout=timeout)
            if packet is None and not message:
                return None
            payload_length = packet[0] & SEQUENCE_LENGTH_BITS
            payload = packet[1:1+payload_length]
            message = message + payload
            # note: we don't actually check the sequence start bit currently, we probably should to
            # check this in case we somehow start reading in the middle of a message
            if packet[0] & SEQUENCE_END_BIT:
                break
        return payload
    def reset(self):
        """ Sends reset message to the MSR605X """
        self.send_message(ESC + b"a")
    def get_firmware_version(self):
        """ Get the firmware version of the connected MSR605X """
        self.send_message(ESC + b"v")
        ret = self.recv_message()
        assert ret[0:1] == ESC
        return ret[1:]
    def flush(self):
        self.buffer = b''
    def write(self, d):
        self.send_message(d)
    def read(self, count=0):
        buf_len = len(self.buffer)
        if count == 0:
            timeout = 100
        else:
            timeout = 0
        if count > buf_len or count == 0:
            new_data = self.recv_message(timeout)
            if not new_data is None:
                self.buffer = self.buffer + new_data
        if count == 0:
            ret = self.buffer
            self.buffer = b''
        else:
            ret = self.buffer[:count]
            self.buffer = self.buffer[count:]
        return ret