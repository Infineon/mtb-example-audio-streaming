""" This is the most simple implementation for streaming serialized structured data from an
embedded target to a host through UART.

In this example, the structured data are audio samples along with some meta-data, see speech_sample.proto.

The structured data are described as Protocol buffers (Protobuf, PB), see https://developers.google.com/protocol-buffers/docs/pythontutorial
For embedded devices programmed in C, a PB implementation exists, see https://jpa.kapsi.fi/nanopb/

Since serialized byte data cannot be transferred over UART without adding some sort of packet
delimiting ("where does a packet of serialized byte start or end?"), such PB-enncoded data
are encoded through 'Consistent Overhead Byte Stuffing' (COBS), see e.g. https://github.com/cmcqueen/cobs-python 
for more details.
For embedded devices programmed in C, a COBS implementation exists, see https://github.com/cmcqueen/cobs-c.
COBS-encoding adds the ability to serialized data to use a specific byte (usual 0x00) as the end
delimiter of a packet.
This allows to read bytes from UART until the delimiter byte is received, then COBS-decode the
accumulated byte array, then PB-decode the serialized byte array back into structured data.

Doing all this right makes sure that you have a single source of truth - the PB description of your
message which is compiled into C and Python code at the same time once you decide to change/adjust
the message content.

The overhead on the embedded target is the code and the runtime for the PB- and COBS encoding and
some intermediate RAM for moving the actual data through the data path.
"""

from host_app.rtstream_common import RTStreaming

rtstreaming = RTStreaming()

rtstreaming.setup_serial_connection()
speech = rtstreaming.get_speech()

while 1:
    ret = rtstreaming.run_streaming()

    if ret == 0:
        print(f"timestamp of received audio packet [ms]: {speech.timestamp}")
        print(f"a few audio samples: 0x{speech.samples[0]:04x} 0x{speech.samples[1]:04x} 0x{speech.samples[127]:04x}")
