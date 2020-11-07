from abc import ABC, abstractmethod
from typing import List
import json


class DataStructure(ABC):
    @staticmethod
    @abstractmethod
    def parse(data: List[int]):
        """
        Creates a `DataStructure <msp.data_structures.data_structure.html>`_ object from the given data
        :param data:
        :return:
        """
        raise NotImplemented

    def __str__(self):
        return self.to_json()

    def to_json(self):
        return json.dumps(self, default=lambda o: o.__dict__, sort_keys=True, indent=4)