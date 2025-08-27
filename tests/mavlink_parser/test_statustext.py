from typing import List, Union
from mavlink_definitions import MAVLinkDefinitions
from mavlink_parser import MAVLink2Parser, MAVLink2Packet, ParseResult

STATUSTEXT_PACKET = bytearray.fromhex(
    "fd1000000d0101fd0000064172647550696c6f742052656164798f5b"
)


def test_statustext_raw():
    parser = MAVLink2Parser()
    results: List[Union[MAVLink2Packet, ParseResult]] = []

    for byte in STATUSTEXT_PACKET:
        result = parser.parse_byte(byte)
        results.append(result)

    assert results[0] == ParseResult.PACKET_STARTED
    assert all(result is ParseResult.NONE for result in results[1:-1])
    assert isinstance(results[-1], MAVLink2Packet)

    packet = results[-1]
    assert packet.raw == STATUSTEXT_PACKET
    assert packet.payload_length == 0x10
    assert packet.incompatibility == 0x00
    assert packet.compatibility == 0x00
    assert packet.sequence == 0x0D
    assert packet.system_id == 0x01
    assert packet.component_id == 0x01
    assert packet.message_id == 0xFD
    assert packet.payload == bytearray.fromhex("064172647550696c6f74205265616479")
    assert packet.checksum == 0x5B8F
    assert packet.signature == bytearray()


def test_statustext_fields():
    parser = MAVLink2Parser(MAVLinkDefinitions())
    packet: Union[MAVLink2Packet, ParseResult] = ParseResult.NONE
    for byte in STATUSTEXT_PACKET:
        packet = parser.parse_byte(byte)

    assert isinstance(packet, MAVLink2Packet)
    assert parser.definitions is not None
    message = parser.definitions.get_message(packet.message_id)
    assert message is not None
    assert message.name == "STATUSTEXT"

    assert len(packet.fields) == 4
    assert packet.fields["severity"] == "MAV_SEVERITY_INFO"
    assert packet.fields["text"] == "ArduPilot Ready"
    assert packet.fields["id"] == 0
    assert packet.fields["chunk_seq"] == 0
