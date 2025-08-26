from typing import List, Union

from pytest import approx
from mavlink_definitions import MAVLinkDefinitions
from mavlink_parser import MAVLink2Parser, MAVLink2Packet, ParseResult

VIBRATION_PACKET = bytearray.fromhex(
    "fd1400000b0101f1000003e642000000000080918c3cfa4b3b3c0b44ce3cc4be"
)


def test_vibration_raw():
    parser = MAVLink2Parser()
    results: List[Union[MAVLink2Packet, ParseResult]] = []

    for byte in VIBRATION_PACKET:
        result = parser.parse_byte(byte)
        results.append(result)

    assert results[0] == ParseResult.PACKET_STARTED
    assert all(result is ParseResult.NONE for result in results[1:-1])
    assert isinstance(results[-1], MAVLink2Packet)

    packet = results[-1]
    assert packet.raw == VIBRATION_PACKET
    assert packet.payload_length == 0x14
    assert packet.incompatibility == 0x00
    assert packet.compatibility == 0x00
    assert packet.sequence == 0x0B
    assert packet.system_id == 0x01
    assert packet.component_id == 0x01
    assert packet.message_id == 0xF1
    assert packet.payload == bytearray.fromhex(
        "03e642000000000080918c3cfa4b3b3c0b44ce3c"
    )
    assert packet.checksum == 0xBEC4
    assert packet.signature == bytearray()


def test_vibration_fields():
    parser = MAVLink2Parser(MAVLinkDefinitions())
    packet: Union[MAVLink2Packet, ParseResult] = ParseResult.NONE
    for byte in VIBRATION_PACKET:
        packet = parser.parse_byte(byte)

    assert isinstance(packet, MAVLink2Packet)
    assert parser.definitions is not None
    message = parser.definitions.get_message(packet.message_id)
    assert message is not None
    assert message.name == "VIBRATION"

    assert len(packet.fields) == 7
    assert packet.fields["time_usec"] == 0x000000000042E603
    assert approx(packet.fields["vibration_x"]) == 0.017159224
    assert approx(packet.fields["vibration_y"]) == 0.011431688
    assert approx(packet.fields["vibration_z"]) == 0.02517893
    assert packet.fields["clipping_0"] == 0
    assert packet.fields["clipping_1"] == 0
    assert packet.fields["clipping_2"] == 0
