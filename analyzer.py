import json
from typing import Dict, List, Optional, Union
from mavlink_definitions import MAVLinkDefinitions, MAVLinkMessage
from mavlink_parser import MAVLink2Packet, MAVLink2Parser, ParseResult
from saleae.analyzers import HighLevelAnalyzer, AnalyzerFrame

MAVLINK_PACKET_TYPE = "MAVLink"
MAVLINK_PACKET_FORMAT = "{{data.message}}"


class MAVLinkAnalyzer(HighLevelAnalyzer):
    def __init__(self):
        self.result_types: Dict[str, Dict[str, str]] = {
            MAVLINK_PACKET_TYPE: {"format": MAVLINK_PACKET_FORMAT}
        }
        self.definitions = MAVLinkDefinitions()
        self.parser = MAVLink2Parser(self.definitions)
        self.packet_start = None

    def decode(self, frame: AnalyzerFrame):
        def get_data() -> bytes:
            return frame.data["data"]  # type: ignore

        frames: List[AnalyzerFrame] = list()
        data = get_data()
        for byte in data:
            result = self.parser.parse_byte(byte)
            if result == ParseResult.PACKET_STARTED:
                self.packet_start = frame.start_time  # type: ignore
            elif isinstance(result, MAVLink2Packet):
                frame = self._generate_frame(result, self.packet_start, frame.end_time)  # type: ignore
                frames.append(frame)

        return frames

    def _packet_to_data(
        self, packet: MAVLink2Packet, message: Optional[MAVLinkMessage]
    ):
        data: Dict[str, Union[str, int]] = {}
        data["message"] = (
            message.name if message is not None else f"id({packet.message_id})"
        )
        route = f"{packet.system_id}:{packet.component_id}"
        if "target_system" in packet.fields and "target_component" in packet.fields:
            route = f"{route} -> {packet.fields['target_system']}:{packet.fields['target_component']}"
        data["route"] = route
        data["system_id"] = str(packet.system_id)
        data["component_id"] = str(packet.component_id)
        data["payload"] = json.dumps(packet.fields)
        return data

    def _generate_frame(
        self, packet: MAVLink2Packet, start_time, end_time  # type: ignore
    ) -> AnalyzerFrame:
        message = self.definitions.get_message(packet.message_id)
        data = self._packet_to_data(packet, message)

        format = MAVLINK_PACKET_TYPE

        return AnalyzerFrame(format, start_time, end_time, data)  # type: ignore
