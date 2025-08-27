from typing import Dict, List, Optional


class MAVLinkMessage:
    def __init__(self):
        self.id: int = 0
        self.name: str = ""
        self.fields: List[MAVLinkMessageField] = []
        self.deprecated: bool = False


class MAVLinkMessageField:
    def __init__(self):
        self.name: str = ""
        self.type: str = ""
        self.enum: Optional[str] = None
        self.extension: bool = False


class MAVLinkEnum:
    def __init__(self):
        self.name: str = ""
        self.values: Dict[int, str] = {}
        self.bitmask: bool = False
        self.deprecated: bool = False
