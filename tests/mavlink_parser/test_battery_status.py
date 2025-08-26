from typing import List, Union
from mavlink_definitions import MAVLinkDefinitions
from mavlink_parser import MAVLink2Parser, MAVLink2Packet, ParseResult

BATTERY_STATUS_PACKET = bytearray.fromhex(
    "fd2900000c01019300000000000000000000ff7fffffffffffffffffffffffffffffffffffffffff0000000000ff00000000066f93"
)


def test_battery_status_raw():
    parser = MAVLink2Parser()
    results: List[Union[MAVLink2Packet, ParseResult]] = []

    for byte in BATTERY_STATUS_PACKET:
        result = parser.parse_byte(byte)
        results.append(result)

    assert results[0] == ParseResult.PACKET_STARTED
    assert all(result is ParseResult.NONE for result in results[1:-1])
    assert isinstance(results[-1], MAVLink2Packet)

    packet = results[-1]
    assert packet.raw == BATTERY_STATUS_PACKET
    assert packet.payload_length == 0x29
    assert packet.incompatibility == 0x00
    assert packet.compatibility == 0x00
    assert packet.sequence == 0x0C
    assert packet.system_id == 0x01
    assert packet.component_id == 0x01
    assert packet.message_id == 0x93
    assert packet.payload == bytearray.fromhex(
        "0000000000000000ff7fffffffffffffffffffffffffffffffffffffffff0000000000ff0000000006"
    )
    assert packet.checksum == 0x936F
    assert packet.signature == bytearray()


def test_battery_status_fields():
    parser = MAVLink2Parser(MAVLinkDefinitions())
    packet: Union[MAVLink2Packet, ParseResult] = ParseResult.NONE
    for byte in BATTERY_STATUS_PACKET:
        packet = parser.parse_byte(byte)

    assert isinstance(packet, MAVLink2Packet)
    assert parser.definitions is not None
    message = parser.definitions.get_message(packet.message_id)
    assert message is not None
    assert message.name == "BATTERY_STATUS"

    assert len(packet.fields) == 14
    assert packet.fields["id"] == 0
    assert packet.fields["battery_function"] == "MAV_BATTERY_FUNCTION_UNKNOWN"
    assert packet.fields["type"] == "MAV_BATTERY_TYPE_UNKNOWN"
    assert packet.fields["temperature"] == 0x7FFF
    assert packet.fields["voltages"] == [
        0xFFFF,
        0xFFFF,
        0xFFFF,
        0xFFFF,
        0xFFFF,
        0xFFFF,
        0xFFFF,
        0xFFFF,
        0xFFFF,
        0xFFFF,
    ]
    assert packet.fields["current_battery"] == 0
    assert packet.fields["current_consumed"] == 0
    assert packet.fields["energy_consumed"] == 0
    assert packet.fields["battery_remaining"] == -1
    assert packet.fields["time_remaining"] == 0
    assert packet.fields["charge_state"] == "MAV_BATTERY_CHARGE_STATE_UNHEALTHY"
    assert packet.fields["voltages_ext"] == [0, 0, 0, 0]
    assert packet.fields["mode"] == "MAV_BATTERY_MODE_UNKNOWN"
    assert packet.fields["fault_bitmask"] == []
