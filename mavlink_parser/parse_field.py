import re
import struct
from typing import Dict, Iterator, List, Sequence, Tuple, Union

from mavlink_definitions import MAVLinkDefinitions, MAVLinkMessageField

ARRAY_FORMAT = r"^(?P<type>.+)\[(?P<length>\d+)\]$"
VALUE_FORMATS = {
    "uint8_t": "B",
    "uint16_t": "H",
    "uint32_t": "I",
    "uint64_t": "Q",
    "int8_t": "b",
    "int16_t": "h",
    "int32_t": "i",
    "int64_t": "q",
    "float": "f",
    "double": "d",
    "uint8_t_mavlink_version": "B",
    "char": "s",
}

FIELD_ORDER = [
    ["uint64_t", "int64_t", "double"],
    ["uint32_t", "int32_t", "float"],
    ["uint16_t", "int16_t"],
    ["uint8_t", "int8_t", "char"],
]


class ValueFormat:
    def __init__(self, raw_format: str, length: int, is_array: bool, is_string: bool):
        self.raw_format = raw_format
        self.length = length
        self.is_array = is_array
        self.is_string = is_string

    _cache: Dict[str, "ValueFormat"] = {}

    @staticmethod
    def get(type: str) -> "ValueFormat":
        if type not in ValueFormat._cache:
            ValueFormat._cache[type] = ValueFormat.parse(type)
        return ValueFormat._cache[type]

    @staticmethod
    def parse(type: str) -> "ValueFormat":
        if type in VALUE_FORMATS:
            return ValueFormat("<" + VALUE_FORMATS[type], 1, False, False)

        match = re.match(ARRAY_FORMAT, type)
        if match is None or match["type"] not in VALUE_FORMATS:
            raise ValueError(f"No format for type={type}")

        array_type = match["type"]
        array_format = VALUE_FORMATS[array_type]
        array_length = int(match["length"])

        full_format = "<" + str(array_length) + array_format
        is_string = array_type == "char"
        is_array = not is_string
        return ValueFormat(full_format, array_length, is_array, is_string)


RawValue = Union[bytes, int, float]
RawValues = Tuple[RawValue, ...]

UnpackedValue = Union[int, float, str, Sequence[Union[str, int]]]
FieldValue = Union[UnpackedValue, Sequence[UnpackedValue]]


def fields_in_order(
    fields: Sequence[MAVLinkMessageField],
) -> Iterator[MAVLinkMessageField]:
    base_fields = [field for field in fields if not field.extension]
    extensions = [field for field in fields if field.extension]
    for types in FIELD_ORDER:
        for field in base_fields:
            if any(field.type.startswith(type) for type in types):
                yield field
    for field in extensions:
        yield field


def parse_field(
    definitions: MAVLinkDefinitions, field: MAVLinkMessageField, data: memoryview
) -> Tuple[memoryview, FieldValue]:
    format = ValueFormat.get(field.type)
    new_data, raw_value = _unpack_raw_value(format.raw_format, data)

    if format.is_string:
        string_bytes = raw_value[0]
        assert isinstance(string_bytes, bytes)
        value = string_bytes.decode("ascii").rstrip("\x00")
        return (new_data, value)
    elif format.is_array:
        values = [_unpack_enum(definitions, field, raw) for raw in raw_value]
        return (new_data, values)
    else:
        values = _unpack_enum(definitions, field, raw_value[0])
        return (new_data, values)


def _unpack_raw_value(
    format: str, data: memoryview
) -> Tuple[memoryview, Tuple[RawValue, ...]]:
    value_length = struct.calcsize(format)
    field_data = data[:value_length]
    if len(field_data) < value_length:
        field_data = bytearray(field_data)
        field_data.extend([0] * (value_length - len(field_data)))
    raw_value = struct.unpack(format, field_data)
    new_data = data[value_length:]
    return (new_data, raw_value)


def _unpack_enum(
    definitions: MAVLinkDefinitions,
    field: MAVLinkMessageField,
    raw_value: RawValue,
) -> UnpackedValue:
    enum = definitions.get_enum(field.enum) if field.enum else None
    if enum is None:
        return raw_value

    assert isinstance(raw_value, int)
    raw_value = int(raw_value)
    if not enum.bitmask:
        return enum.values.get(raw_value, raw_value)
    else:
        result: List[Union[str, int]] = []
        for bit in range(raw_value.bit_length()):
            bit_value = 1 << bit
            if raw_value & bit_value:
                enum_name = enum.values.get(bit_value, bit_value)
                result.append(enum_name)
        return result
