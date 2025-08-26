from abc import abstractmethod
from enum import Enum
from typing import Dict, Optional, TypeVar, Union, Generic, Type

from mavlink_definitions import MAVLinkDefinitions

from .parse_field import FieldValue, fields_in_order, parse_field


class MAVLinkPacket:
    def __init__(self):
        self.raw: bytearray = bytearray()
        self.payload_length: int = 0
        self.sequence: int = 0
        self.system_id: int = 0
        self.component_id: int = 0
        self.message_id: int = 0
        self.payload: bytearray = bytearray()
        self.checksum: int = 0
        self.fields: Dict[str, FieldValue] = {}


class ParseResult(Enum):
    NONE = 1
    PACKET_STARTED = 2


class ParseByteResult(Enum):
    NONE = 1
    PACKET_STARTED = 2
    PACKET_READY = 3


TPacket = TypeVar("TPacket", bound=MAVLinkPacket)


class MAVLinkParserBase(Generic[TPacket]):
    def __init__(
        self,
        packet_cls: Type[TPacket],
        definitions: Optional[MAVLinkDefinitions] = None,
    ):
        super().__init__()
        self.packet = packet_cls()
        self.packet_cls = packet_cls
        self.definitions = definitions

    def parse_byte(self, byte: int) -> Union[TPacket, ParseResult]:
        byte_result = self._parse_byte_impl(byte)
        if byte_result == ParseByteResult.PACKET_STARTED:
            return ParseResult.PACKET_STARTED

        if byte_result == ParseByteResult.PACKET_READY:
            self._deserialize_fields()
            return self.packet

        return ParseResult.NONE

    @abstractmethod
    def _parse_byte_impl(self, byte: int) -> ParseByteResult: ...

    def _deserialize_fields(self):
        if self.definitions is None:
            return
        message = self.definitions.get_message(self.packet.message_id)
        if message is None:
            return

        data = memoryview(self.packet.payload)
        fields = fields_in_order(message.fields)
        for field in fields:
            data, value = parse_field(self.definitions, field, data)
            self.packet.fields[field.name] = value
