from typing import List, Union
from mavlink_definitions import MAVLinkDefinitions
from mavlink_parser import MAVLink2Parser, MAVLink2Packet, ParseResult

HEARTBEAT_PACKET = bytearray.fromhex("fd090000cf0101000000040000000a030105037950")


def test_heartbeat_raw():
    parser = MAVLink2Parser()
    results: List[Union[MAVLink2Packet, ParseResult]] = []

    for byte in HEARTBEAT_PACKET:
        result = parser.parse_byte(byte)
        results.append(result)

    assert results[0] == ParseResult.PACKET_STARTED
    assert all(result is ParseResult.NONE for result in results[1:-1])
    assert isinstance(results[-1], MAVLink2Packet)

    packet = results[-1]
    assert packet.raw == HEARTBEAT_PACKET
    assert packet.payload_length == 0x09
    assert packet.incompatibility == 0x00
    assert packet.compatibility == 0x00
    assert packet.sequence == 0xCF
    assert packet.system_id == 0x01
    assert packet.component_id == 0x01
    assert packet.message_id == 0x000000
    assert packet.payload == bytearray.fromhex("040000000a03010503")
    assert packet.checksum == 0x5079
    assert packet.signature == bytearray()


def test_heartbeat_fields():
    parser = MAVLink2Parser(MAVLinkDefinitions())
    packet: Union[MAVLink2Packet, ParseResult] = ParseResult.NONE
    for byte in HEARTBEAT_PACKET:
        packet = parser.parse_byte(byte)

    assert isinstance(packet, MAVLink2Packet)
    assert parser.definitions is not None
    message = parser.definitions.get_message(packet.message_id)
    assert message is not None
    assert message.name == "HEARTBEAT"

    assert len(packet.fields) == 6
    assert packet.fields["type"] == "MAV_TYPE_GROUND_ROVER"
    assert packet.fields["autopilot"] == "MAV_AUTOPILOT_ARDUPILOTMEGA"
    assert packet.fields["base_mode"] == ["MAV_MODE_FLAG_CUSTOM_MODE_ENABLED"]
    assert packet.fields["custom_mode"] == 0x4
    assert packet.fields["system_status"] == "MAV_STATE_CRITICAL"
    assert packet.fields["mavlink_version"] == 3
