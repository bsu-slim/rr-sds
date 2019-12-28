"""
This module defines the information units concerning abstract concepts.
"""
from retico.core import abstract


class DialogueActIU(abstract.IncrementalUnit):
    """A Dialog Act Incremental Unit.

    This IU represents a Dialogue Act together with concepts and their
    values. In this implementation only a single act can be expressed with a
    single IU.

    Attributes:
        act (string): A representation of the current act as a string.
        concepts (dict): A dictionary of names of concepts being mapped on to
            their actual values.
    """

    @staticmethod
    def type():
        return "Dialogue Act Incremental Unit"

    def __init__(self, creator=None, iuid=0, previous_iu=None, grounded_in=None,
                 payload=None, act=None, concepts=None, **kwargs):
        """Initialize the DialogueActIU with act and concepts.

        Args:
            act (string): A representation of the act.
            concepts (dict): A representation of the concepts as a dictionary.
        """
        super().__init__(creator=creator, iuid=iuid, previous_iu=previous_iu,
                         grounded_in=grounded_in, payload=payload)
        self.act = act
        self.concepts = {}
        if concepts:
            self.concepts = concepts
        self.confidence = 0.0

    def set_act(self, act, concepts=None, confidence=1.0):
        """Set the act and concept of the IU.

        Old acts or concepts will be overwritten.

        Args:
            act (string): The act of the IU as a string.
            concepts (dict): A dictionary containing the new concepts.
            confidence (float): Confidence of the act prediction
        """
        self.act = act
        if concepts:
            self.concepts = concepts
        self.confidence = confidence
        self.payload = {'act':act, 'concepts':concepts, 'confidence':confidence}


class DispatchableActIU(DialogueActIU):
    """A Dialogue Act Incremental Unit that can has the information if it should
    be dispatched once it has been transformed into speech.

    Attributes:
        dispatch (bool): Whether the speech resulting from this IU should be
            dispatched or not.
    """

    def __init__(self, dispatch=False, **kwargs):
        super().__init__(**kwargs)
        self.dispatch = dispatch


class DialogueDecisionIU(abstract.IncrementalUnit):
    """A Dialogue Manager Decision.

    This IU represents a Dialogue Decision together with concepts and their
    values. In this implementation only a single decision can be expressed with a
    single IU.

    Attributes:
        decision (string): A representation of the current act as a string.
        concepts (dict): A dictionary of names of concepts being mapped on to
            their actual values.
    """

    @staticmethod
    def type():
        return "Dialogue Decision Incremental Unit"

    def __init__(self, creator=None, iuid=0, previous_iu=None, grounded_in=None,
                 payload=None, decision=None, concepts=None, **kwargs):
        """Initialize the DialogueDecisionIU with decision.

        Args:
            act (string): A representation of the act.
            concepts (dict): A representation of the concepts as a dictionary.
        """
        super().__init__(creator=creator, iuid=iuid, previous_iu=previous_iu,
                         grounded_in=grounded_in, payload=payload)
        self.decision = decision
        self.concepts = {}
        if concepts:
            self.concepts = concepts
        self.confidence = 0.0

    def set_act(self, decision, concepts=None, confidence=1.0):
        """Set the act and concept of the IU.

        Old acts or concepts will be overwritten.

        Args:
            act (string): The act of the IU as a string.
            concepts (dict): A dictionary containing the new concepts.
            confidence (float): Confidence of the act prediction
        """
        self.decision = decision
        if concepts:
            self.concepts = concepts
        self.confidence = confidence
        self.payload = {'decision':decision, 'concepts':concepts}
