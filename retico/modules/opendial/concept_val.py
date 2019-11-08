import sys
import os
sys.path.append(os.environ['PYOD'])

from bn.values.double_val import DoubleVal
from bn.values.value import Value
from bn.values.none_val import NoneVal

from retico.core.abstract import IncrementalUnit

class ConceptVal (Value):

    def __init__(self, concept, value, confidence):
        self._concept = concept
        self._value = value
        self._confidence = confidence

    def __hash__(self):
        """
        Returns the hashcode for the double.

        :return: the hashcode
        """
        return hash(self._concept)

    def __eq__(self, other):

        if isinstance(other, NoneVal):
            return False

        if not isinstance(other, DoubleVal):
            return other._concept == self._concept

        if abs(self._confidence - other.get_double()) > DoubleVal.eps:
            return False

        return True

    def __lt__(self, other):
        """
        Compares the double value to another value.

        :param other: the object to compare
        :return: usual ordering, or hashcode difference if the value is not a double
        """
        if isinstance(other, NoneVal):
            return False

        if not isinstance(other, DoubleVal):
            return False
        return self._confidence < other.get_double()

    def __contains__(self, item):
        return item._concept in self._concept

    def __copy__(self):
        return ConceptVal(self._concept, self._value, self._confidence)

    # @dispatch(Value)
    # def concatenate(value):
    #     return None
        


