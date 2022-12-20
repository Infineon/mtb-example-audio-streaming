

from collections import deque
from dataclasses import dataclass   # used to create some simple container for statistics

import serial
from serial.tools import list_ports

from cobs import cobs  # smart binary serial encoding and decoding
from google.protobuf import message

from host_app.speech_sample_pb2 import SpeechSample

BAUDRATE_KIT = 1000000
# COBS 1-byte delimiter is hex zero as a (binary) bytes character
ZERO_BYTE = b'\x00'

@dataclass
class TransferStats:
    """A class to collect statistics about the speech packets we receive.

    Data points we gather from the serial communication of speech packets we
    receive from the embedded device.
    """

    # based on (decoded) message content of Protobuf message
    time_stamps: deque = deque([0, 0, 0, 0], maxlen=4)
    avg_delta_time_stamp: float = 0.0  # see update_avg_delta

    # based on (encoded, decoded) COBS packets
    num_zero_len_cobs: int = 0
    faulty_first_byte: int = 0
    # the max len in number of bytes of the so-far received COBS packets
    max_len_cobs: int = 0

    def update_avg_delta(self):
        """Update the average timestamp delta.

        Take the latest and last but one timestamp,
        calc the delta and update the global average.
        """
        temp = list(self.time_stamps)
        # time_stamps[-1] is latest =>
        # temp[len-1] - temp[len-2] is latest delta
        delta = temp[len(temp)-1] - temp[len(temp)-2]
        self.avg_delta_time_stamp = (self.avg_delta_time_stamp + delta)/2.0

class RTStreaming():
    def __init__(self):
        self.ser = None
        self.serial_stats = TransferStats()
        self.speech = SpeechSample()
        # keep a history of the last 4 0x00-delimited byte arrays for debugging
        self.q_enc_blobs = deque(maxlen=4)
        self.q_recv_bytes = deque(maxlen=4)

    def setup_serial_connection(self):
        # Grep through the available serial ports and choose the one which is
        # a KitProg3. Assumes that only one single KitProg3 device is attached.
        try:
            kit = next(list_ports.grep("KitProg3"))
            print(f"Using {kit.description}, identified as {kit.hwid}")
        except StopIteration:
            print("No KitProg3 USB device found")
            exit(1)

        try:
            self.ser = serial.Serial(kit.device,
                                     baudrate=BAUDRATE_KIT,
                                     bytesize=8,
                                     parity='N',
                                     stopbits=1,
                                     timeout=3)
        except serial.SerialException:
            print("unable to connect to KitProg3 USB device ...")
            exit(1)

        return 0

    def close_serial_connection(self):
        self.ser.close()

    def run_streaming(self):
        # COBS ensures the zero-byte is *only* used as the packet-end delimiter
        # read until the COBS packet ending delimiter is found
        recv_bytes = self.ser.read_until(ZERO_BYTE)
        n = len(recv_bytes)

        if n > 0:
            # take everything except the trailing zero byte, b'\x00'
            encoded_blob = recv_bytes[:(n - 1)]
            self.q_recv_bytes.append(recv_bytes)
            self.q_enc_blobs.append(encoded_blob)

            try:
                # recover binary data encoded on embedded target
                decoded_blob = cobs.decode(encoded_blob)
            except cobs.DecodeError as exc:
                if str(exc) == "not enough input bytes for length code":
                    return -1  # Cobs error
                else:
                    # ignore all the other exceptions for the moment.
                    print(exc)
            # we have decoded a 'good' blob
            else:
                if len(encoded_blob) > self.serial_stats.max_len_cobs:
                    self.serial_stats.max_len_cobs = len(encoded_blob)

                try:
                    self.speech.ParseFromString(decoded_blob)
                except message.DecodeError as exc:
                    if str(exc) == "Error parsing message with type 'SpeechSample'":
                        return -1  # Protobuf error
                    else:
                        # ignore all the other exceptions for the moment.
                        print(exc)
                else:
                    self.serial_stats.time_stamps.append(self.speech.timestamp)
                    self.serial_stats.update_avg_delta()

        return 0

    def get_speech(self):
        return self.speech

    def get_serial_stats(self):
        return self.serial_stats
