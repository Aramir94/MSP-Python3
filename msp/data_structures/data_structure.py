from abc import ABC, abstractmethod
from typing import List


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

    @abstractmethod
    def to_array(self) -> List[int]:
        raise NotImplemented
