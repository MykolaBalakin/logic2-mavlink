from typing import Dict, Union
from .parser import MAVLINK_DEFINITION_DEFAULT, MAVLinkDefinitionsParser
from .types import MAVLinkEnum, MAVLinkMessage


class MAVLinkDefinitions:
    def __init__(self, filename: str = MAVLINK_DEFINITION_DEFAULT):
        self._initialized = False
        self._messages: Dict[Union[int, str], MAVLinkMessage] = {}
        self._enums: Dict[str, MAVLinkEnum] = {}
        self.filename = filename

    def init(self):
        self._ensure_initialized()

    def get_message(self, idOrName: Union[int, str]):
        self._ensure_initialized()
        return self._messages.get(idOrName, None)

    def get_enum(self, name: str):
        self._ensure_initialized()
        return self._enums.get(name, None)

    def _ensure_initialized(self):
        if self._initialized:
            return

        parser = MAVLinkDefinitionsParser()
        parser.parse(self.filename)
        messages_by_id = {message.id: message for message in parser.messages}
        messages_by_name = {message.name: message for message in parser.messages}
        self._messages = {**messages_by_id, **messages_by_name}
        self._enums = {enum.name: enum for enum in parser.enums}
        self._initialized = True
