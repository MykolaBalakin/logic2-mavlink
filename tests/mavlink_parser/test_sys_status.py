from typing import List, Union
from mavlink_definitions import MAVLinkDefinitions
from mavlink_parser import MAVLink2Parser, MAVLink2Packet, ParseResult

SYS_STATUS_PACKET = bytearray.fromhex(
    "fd1f0000d201010100000fdc30130f8020120b8110031a000200730000000000000000000000000060b166"
)


def test_sys_status_raw():
    parser = MAVLink2Parser()
    results: List[Union[MAVLink2Packet, ParseResult]] = []

    for byte in SYS_STATUS_PACKET:
        result = parser.parse_byte(byte)
        results.append(result)

    assert results[0] == ParseResult.PACKET_STARTED
    assert all(result is ParseResult.NONE for result in results[1:-1])
    assert isinstance(results[-1], MAVLink2Packet)

    packet = results[-1]
    assert packet.raw == SYS_STATUS_PACKET
    assert packet.payload_length == 0x1F
    assert packet.incompatibility == 0x00
    assert packet.compatibility == 0x00
    assert packet.sequence == 0xD2
    assert packet.system_id == 0x01
    assert packet.component_id == 0x01
    assert packet.message_id == 0x01
    assert packet.payload == bytearray.fromhex(
        "0fdc30130f8020120b8110031a000200730000000000000000000000000060"
    )
    assert packet.checksum == 0x66B1
    assert packet.signature == bytearray()


def test_sys_status_fields():
    parser = MAVLink2Parser(MAVLinkDefinitions())
    packet: Union[MAVLink2Packet, ParseResult] = ParseResult.NONE
    for byte in SYS_STATUS_PACKET:
        packet = parser.parse_byte(byte)

    assert isinstance(packet, MAVLink2Packet)
    assert parser.definitions is not None
    message = parser.definitions.get_message(packet.message_id)
    assert message is not None
    assert message.name == "SYS_STATUS"

    assert len(packet.fields) == 16
    assert packet.fields["onboard_control_sensors_present"] == [
        "MAV_SYS_STATUS_SENSOR_3D_GYRO",
        "MAV_SYS_STATUS_SENSOR_3D_ACCEL",
        "MAV_SYS_STATUS_SENSOR_3D_MAG",
        "MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE",
        "MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL",
        "MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION",
        "MAV_SYS_STATUS_SENSOR_YAW_POSITION",
        "MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL",
        "MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS",
        "MAV_SYS_STATUS_GEOFENCE",
        "MAV_SYS_STATUS_AHRS",
        "MAV_SYS_STATUS_LOGGING",
        "MAV_SYS_STATUS_SENSOR_BATTERY",
        "MAV_SYS_STATUS_PREARM_CHECK",
    ]
    assert packet.fields["onboard_control_sensors_enabled"] == [
        "MAV_SYS_STATUS_SENSOR_3D_GYRO",
        "MAV_SYS_STATUS_SENSOR_3D_ACCEL",
        "MAV_SYS_STATUS_SENSOR_3D_MAG",
        "MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE",
        "MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS",
        "MAV_SYS_STATUS_AHRS",
        "MAV_SYS_STATUS_SENSOR_BATTERY",
        "MAV_SYS_STATUS_PREARM_CHECK",
    ]
    assert packet.fields["onboard_control_sensors_health"] == [
        "MAV_SYS_STATUS_SENSOR_3D_GYRO",
        "MAV_SYS_STATUS_SENSOR_3D_ACCEL",
        "MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE",
        "MAV_SYS_STATUS_SENSOR_LASER_POSITION",
        "MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS",
        "MAV_SYS_STATUS_GEOFENCE",
        "MAV_SYS_STATUS_LOGGING",
        "MAV_SYS_STATUS_SENSOR_BATTERY",
    ]
    assert packet.fields["load"] == 26
    assert packet.fields["voltage_battery"] == 2
    assert packet.fields["current_battery"] == 115
    assert packet.fields["battery_remaining"] == 96
    assert packet.fields["drop_rate_comm"] == 0
    assert packet.fields["errors_comm"] == 0
    assert packet.fields["errors_count1"] == 0
    assert packet.fields["errors_count2"] == 0
    assert packet.fields["errors_count3"] == 0
    assert packet.fields["errors_count4"] == 0
    assert packet.fields["onboard_control_sensors_present_extended"] == []
    assert packet.fields["onboard_control_sensors_enabled_extended"] == []
    assert packet.fields["onboard_control_sensors_health_extended"] == []
