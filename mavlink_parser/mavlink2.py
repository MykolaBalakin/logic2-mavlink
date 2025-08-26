from enum import Enum
from typing import Callable, Dict, Optional

from mavlink_definitions import MAVLinkDefinitions
from .base import MAVLinkPacket, MAVLinkParserBase, ParseByteResult


class MAVLink2Packet(MAVLinkPacket):
    def __init__(self):
        super().__init__()
        self.incompatibility: int = 0
        self.compatibility: int = 0
        self.signature: bytearray = bytearray()


class MAVLink2Parser(MAVLinkParserBase[MAVLink2Packet]):
    class State(Enum):
        HEADER = 1
        LENGTH = 2
        INC_FLAGS = 3
        CMP_FLAGS = 4
        SEQ = 5
        SYS_ID = 6
        COMP_ID = 7
        MSG_ID = 8
        PAYLOAD = 9
        CHECKSUM = 10
        SIGNATURE = 11
        PACKET_READY = 13

    def __init__(self, definitions: Optional[MAVLinkDefinitions] = None):
        super().__init__(MAVLink2Packet, definitions)
        self.state: MAVLink2Parser.State = self.State.HEADER
        self.state_handlers: Dict[
            "MAVLink2Parser.State",
            Callable[[int], "MAVLink2Parser.State"],
        ] = {
            self.State.HEADER: self._parse_header,
            self.State.LENGTH: self._parse_length,
            self.State.INC_FLAGS: self._parse_inc_flags,
            self.State.CMP_FLAGS: self._parse_cmp_flags,
            self.State.SEQ: self._parse_seq,
            self.State.SYS_ID: self._parse_sys_id,
            self.State.COMP_ID: self._parse_comp_id,
            self.State.MSG_ID: self._parse_msg_id,
            self.State.PAYLOAD: self._parse_payload,
            self.State.CHECKSUM: self._parse_checksum,
            self.State.SIGNATURE: self._parse_signature,
        }
        self.packet: MAVLink2Packet = MAVLink2Packet()

    def _parse_byte_impl(self, byte: int) -> ParseByteResult:
        state = self.state
        handler = self.state_handlers.get(state)
        if handler is None:
            raise ValueError(f"Invalid state={state}")
        state = handler(byte)

        self.state = state

        if state == MAVLink2Parser.State.LENGTH:
            return ParseByteResult.PACKET_STARTED

        if state == MAVLink2Parser.State.PACKET_READY:
            self.state = MAVLink2Parser.State.HEADER
            return ParseByteResult.PACKET_READY

        return ParseByteResult.NONE

    def _parse_header(self, byte: int) -> State:
        if byte == 0xFD:
            self.packet = MAVLink2Packet()
            self.packet.raw.append(byte)
            return MAVLink2Parser.State.LENGTH
        else:
            return MAVLink2Parser.State.HEADER

    def _parse_length(self, byte: int) -> State:
        self.packet.raw.append(byte)
        self.packet.payload_length = byte
        return MAVLink2Parser.State.INC_FLAGS

    def _parse_inc_flags(self, byte: int) -> State:
        self.packet.raw.append(byte)
        self.packet.incompatibility = byte
        return MAVLink2Parser.State.CMP_FLAGS

    def _parse_cmp_flags(self, byte: int) -> State:
        self.packet.raw.append(byte)
        self.packet.compatibility = byte
        return MAVLink2Parser.State.SEQ

    def _parse_seq(self, byte: int) -> State:
        self.packet.raw.append(byte)
        self.packet.sequence = byte
        return MAVLink2Parser.State.SYS_ID

    def _parse_sys_id(self, byte: int) -> State:
        self.packet.raw.append(byte)
        self.packet.system_id = byte
        return MAVLink2Parser.State.COMP_ID

    def _parse_comp_id(self, byte: int) -> State:
        self.packet.raw.append(byte)
        self.packet.component_id = byte
        return MAVLink2Parser.State.MSG_ID

    def _parse_msg_id(self, byte: int) -> State:
        self.packet.raw.append(byte)
        msg_id_len = len(self.packet.raw) - 7
        shift = (msg_id_len - 1) * 8
        self.packet.message_id |= byte << shift
        if msg_id_len < 3:
            return MAVLink2Parser.State.MSG_ID
        else:
            return MAVLink2Parser.State.PAYLOAD

    def _parse_payload(self, byte: int) -> State:
        self.packet.raw.append(byte)
        self.packet.payload.append(byte)
        if len(self.packet.payload) < self.packet.payload_length:
            return MAVLink2Parser.State.PAYLOAD
        else:
            return MAVLink2Parser.State.CHECKSUM

    def _parse_checksum(self, byte: int) -> State:
        self.packet.raw.append(byte)
        checksum_len = len(self.packet.raw) - 10 - self.packet.payload_length
        shift = (checksum_len - 1) * 8
        self.packet.checksum |= byte << shift
        if checksum_len < 2:
            return MAVLink2Parser.State.CHECKSUM
        elif self.packet.incompatibility & 0x1 == 0x1:
            return MAVLink2Parser.State.SIGNATURE
        else:
            return MAVLink2Parser.State.PACKET_READY

    def _parse_signature(self, byte: int) -> State:
        self.packet.raw.append(byte)
        self.packet.signature.append(byte)
        if len(self.packet.signature) < 13:
            return MAVLink2Parser.State.SIGNATURE
        else:
            return MAVLink2Parser.State.PACKET_READY
