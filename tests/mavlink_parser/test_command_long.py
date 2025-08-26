from typing import List, Union
from mavlink_definitions import MAVLinkDefinitions
from mavlink_parser import MAVLink2Parser, MAVLink2Packet, ParseResult

COMMAND_LONG_PACKET = bytearray.fromhex(
    "fd200000b0ffbe4c00000000a0400080bb440000000000000000000000000000000000000000b70001be4966"
)


def test_command_long_raw():
    parser = MAVLink2Parser()
    results: List[Union[MAVLink2Packet, ParseResult]] = []

    for byte in COMMAND_LONG_PACKET:
        result = parser.parse_byte(byte)
        results.append(result)

    assert results[0] == ParseResult.PACKET_STARTED
    assert all(result is ParseResult.NONE for result in results[1:-1])
    assert isinstance(results[-1], MAVLink2Packet)

    packet = results[-1]
    assert packet.raw == COMMAND_LONG_PACKET
    assert packet.payload_length == 0x20
    assert packet.incompatibility == 0x00
    assert packet.compatibility == 0x00
    assert packet.sequence == 0xB0
    assert packet.system_id == 0xFF
    assert packet.component_id == 0xBE
    assert packet.message_id == 0x4C
    assert packet.payload == bytearray.fromhex(
        "0000a0400080bb440000000000000000000000000000000000000000b70001be"
    )
    assert packet.checksum == 0x6649
    assert packet.signature == bytearray()


def test_command_long_fields():
    parser = MAVLink2Parser(MAVLinkDefinitions())
    packet: Union[MAVLink2Packet, ParseResult] = ParseResult.NONE
    for byte in COMMAND_LONG_PACKET:
        packet = parser.parse_byte(byte)

    assert isinstance(packet, MAVLink2Packet)
    assert parser.definitions is not None
    message = parser.definitions.get_message(packet.message_id)
    assert message is not None
    assert message.name == "COMMAND_LONG"

    assert len(packet.fields) == 11
    assert packet.fields["target_system"] == 1
    assert packet.fields["target_component"] == 0xBE
    assert packet.fields["command"] == "MAV_CMD_DO_SET_SERVO"
    assert packet.fields["confirmation"] == 0
    assert packet.fields["param1"] == 5.0
    assert packet.fields["param2"] == 1500.0
    assert packet.fields["param3"] == 0
    assert packet.fields["param4"] == 0
    assert packet.fields["param5"] == 0
    assert packet.fields["param6"] == 0
    assert packet.fields["param7"] == 0
