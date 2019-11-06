"""A module for Dialogue Management provided by PyOpenDial"""
from multipledispatch import dispatch

# retico
from retico.core import abstract
from retico.core.dialogue.common import DialogueActIU
from retico.core.dialogue.common import DialogueDecisionIU
from retico.modules.opendial.concept_val import ConceptVal

# opendial
import sys
sys.path.append("/home/casey/git/PyOpenDial")
from dialogue_system import DialogueSystem
from datastructs.assignment import Assignment
from modules.simulation.simulator import Simulator
from readers.xml_domain_reader import XMLDomainReader
from bn.distribs.distribution_builder import CategoricalTableBuilder
from dialogue_state import DialogueState
from modules.module import Module
from collections import Collection


class OpenDialModule(abstract.AbstractModule, Module):
    """The OpenDial module for Dialogue Management.

    Attributes:
        domain_dir (str): The path to the directory of the domain model (XML) for OpenDial
    """

    @staticmethod
    def name():
        return "OpenDial DM Module"

    @staticmethod
    def description():
        return "A Module providing Dialogue Management provided by OpenDial"

    @staticmethod
    def input_ius():
        return [DialogueActIU]

    @staticmethod
    def output_iu():
        return DialogueDecisionIU

    def __init__(self, domain_dir, **kwargs):
        """Initializes the OpenDialModule.

        Args:
            domain_dir (str): The path to the directory of the domain model (XML) for OpenDial
        """
        super().__init__(**kwargs)
        self.domain_dir = domain_dir
        self._system = DialogueSystem()
        self.cache = None
        self._paused = True
        self._input_iu = None

    def process_iu(self, input_iu):
        self._input_iu = input_iu
        act, concepts = input_iu.payload
        confidence = input_iu.confidence
        # print('dm add({},{})'.format(input_iu.payload, confidence))

        # TODO: incremental
        # TODO: partially observed
        self._system.add_content('intent', ConceptVal('intent',act,confidence))

        for concept in concepts: # TODO: get these concepts to come through the opendial config
            if '_confidence' in concept: continue
            conf = concepts['{}_confidence'.format(concept)]
            self._system.add_content('concept', ConceptVal(concept, str(concepts[concept]), conf))
        
        # if not self._system.is_paused:
        #     for concept in concepts:
        #         iuval = IUVal(str(concepts[concept]), concepts['{}_confidence'.format(concept)])
        #         self._cur_state.add_to_state(Assignment(concept, iuval))
        #     self._system.update() # update once after everything is in

        return None # how to get a result from an attached output module?

    def process_revoke(self, revoked_iu):
        print('dm revoke({})'.format(revoked_iu.payload))


    def setup(self):
        self._system.change_domain(XMLDomainReader.extract_domain(self.domain_dir))
        self._system.change_settings(self._system.get_settings())
        self._system.attach_module(self) # modules can read the dialogue state
        self._system.start_system()

    '''
    Everything below here belongs to the OpenDial Module 
    '''

    def start(self):
        """
        Starts the module.
        """
        self._paused = False

    @dispatch(DialogueState, Collection)
    def trigger(self, state, update_vars):
        if 'decision' in update_vars and state.has_chance_node('decision'):
            action = str(state.query_prob('decision').get_best())
            concept_val = state.query_prob('concept').get_best()
            if isinstance(concept_val, ConceptVal):
                output_iu = self.create_iu(self._input_iu)
                output_iu.set_act(action, {concept_val._concept:concept_val._value}, concept_val._confidence)
                self.append(output_iu)

    @dispatch(bool)
    def pause(self, to_pause):
        """
        Pauses the module.

        :param to_pause: whether to pause the module or not
        """
        self._paused = to_pause

    def is_running(self):
        """
        Returns whether the module is currently running or not.

        :return: whether the module is running or not.
        """
        return not self._paused