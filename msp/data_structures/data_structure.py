from abc import ABC, abstractmethod
from typing import List
import json

from msp.message_ids import MessageIDs


class DataStructure(ABC):
    def __init__(self, code: MessageIDs):
        self.code: MessageIDs = code

    def __str__(self):
        return self.to_json()

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=4)

    @staticmethod
    @abstractmethod
    def parse(data: List[int]):
        """
        Creates a `DataStructure <msp.data_structures.data_structure.html>`_ object from the given data
        :param data:
        :return:
        """
        raise NotImplemented

    @staticmethod
    def get_header() -> bytes:
        return '$'.encode('utf-8') + 'M'.encode('utf-8') + '<'.encode('utf-8')

    @staticmethod
    def perform_checksum(data: bytes) -> bytes:
        checksum = 0
        for i in data:
            checksum = checksum ^ i
        return checksum.to_bytes(1, 'little')

    def serialize(self, setter=False) -> bytes:
        """

        :param setter:
        :return:
        """
        # Serialize Data
        result = int(0).to_bytes(1, 'little')
        result += int(self.code).to_bytes(1, 'little')

        # Serialize Checksum
        result = DataStructure.get_header() + result + DataStructure.perform_checksum(result)

        # Return result
        return result

    class ChecksumMismatch(Exception):
        def __init__(self, code, expected_chksum, actual_chksum):
            self.code = code
            self.expected_chksum = expected_chksum
            self.actual_chksum = actual_chksum