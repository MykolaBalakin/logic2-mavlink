import os
from typing import List, Set
from xml.etree import ElementTree
from .types import MAVLinkEnum, MAVLinkMessage, MAVLinkMessageField


MAVLINK_DEFINITION_PATH = os.path.join(
    os.path.dirname(__file__), "..", "mavlink", "message_definitions", "v1.0"
)
MAVLINK_DEFINITION_DEFAULT = "common.xml"


def get_all_mavlink_definitions():
    all_entries = os.listdir(MAVLINK_DEFINITION_PATH)
    return [
        filename
        for filename in all_entries
        if filename.endswith(".xml")
        and os.path.isfile(os.path.join(MAVLINK_DEFINITION_PATH, filename))
    ]


class MAVLinkDefinitionsParser:

    def __init__(self):
        self._parsed_files: Set[str] = set()
        self.messages: List[MAVLinkMessage] = []
        self.enums: List[MAVLinkEnum] = []

    def parse(self, filename: str):
        if filename in self._parsed_files:
            return
        self._parsed_files.add(filename)
        self._parse_file(filename)

    def _parse_file(self, filename: str):
        xml_tree = ElementTree.parse(os.path.join(MAVLINK_DEFINITION_PATH, filename))
        xml = xml_tree.getroot()
        assert xml.tag == "mavlink"
        for child in xml:
            if child.tag == "version" or child.tag == "dialect":
                pass
            elif child.tag == "include":
                self._parse_include(child)
            elif child.tag == "enums":
                self._parse_enums(child)
            elif child.tag == "messages":
                self._parse_messages(child)
            else:
                raise ValueError(f"Unknown tag={child.tag}")

    def _parse_include(self, xml: ElementTree.Element) -> None:
        assert xml.tag == "include"
        if xml.text is None:
            raise ValueError("Include tag has no filename specified")
        self._parse_file(xml.text)

    def _parse_enums(self, xml: ElementTree.Element) -> None:
        assert xml.tag == "enums"
        for child in xml:
            self._parse_enum(child)

    def _parse_enum(self, xml: ElementTree.Element) -> None:
        assert xml.tag == "enum"
        enum = MAVLinkEnum()
        enum.name = xml.attrib["name"]
        enum.bitmask = xml.attrib.get("bitmask", None) == "true"
        self.enums.append(enum)
        for child in xml:
            if child.tag == "description":
                pass
            elif child.tag == "deprecated":
                enum.deprecated = True
            elif child.tag == "entry":
                enum.values[int(child.attrib["value"])] = child.attrib["name"]
            else:
                raise ValueError(f"Unknown tag={child.tag}")

    def _parse_messages(self, xml: ElementTree.Element) -> None:
        assert xml.tag == "messages"
        for child in xml:
            self._parse_message(child)

    def _parse_message(self, xml: ElementTree.Element) -> None:
        assert xml.tag == "message"
        message = MAVLinkMessage()
        message.id = int(xml.attrib["id"])
        message.name = xml.attrib["name"]
        self.messages.append(message)
        extensions = False
        for child in xml:
            if child.tag == "description" or child.tag == "wip":
                pass
            elif child.tag == "extensions":
                extensions = True
            elif child.tag == "deprecated":
                message.deprecated = True
            elif child.tag == "field":
                field = MAVLinkMessageField()
                field.name = child.attrib["name"]
                field.type = child.attrib["type"]
                field.enum = child.attrib.get("enum", None)
                field.extension = extensions
                message.fields.append(field)
            else:
                raise ValueError(f"Unknown tag={child.tag}")
