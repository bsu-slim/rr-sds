"""A module for Dialogue Management provided by PyOpenDial"""
from multipledispatch import dispatch

# retico
from retico.core import abstract
from retico.core.abstract import IncrementalUnit
from retico.core.dialogue.common import DialogueActIU
from retico.core.dialogue.common import DialogueDecisionIU
from retico.modules.opendial.concept_val import ConceptVal
from retico.core.robot.common import RobotStateIU

# opendial
import sys
import os
sys.path.append(os.environ['PYOD'])
from dialogue_system import DialogueSystem
from datastructs.assignment import Assignment
from modules.simulation.simulator import Simulator
from readers.xml_domain_reader import XMLDomainReader
from bn.distribs.distribution_builder import CategoricalTableBuilder
from dialogue_state import DialogueState
from modules.module import Module
from collections import Collection
from bn.values.none_val import NoneVal


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
        return [DialogueActIU,IncrementalUnit]

    @staticmethod
    def output_iu():
        return DialogueDecisionIU

    def __init__(self, domain_dir, variables=None, **kwargs):
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
        self._variables = variables
        self._prior_state = {}

    # def new_utterance(self):
    #     self._system.add_content('concept', ConceptVal('','',0)) 
    #     super().new_utterance()

    def process_iu(self, input_iu):

        if type(input_iu) == DialogueActIU:
            self._input_iu = input_iu
            act = input_iu.payload['act']
            concepts = input_iu.payload['concepts']
            confidence = input_iu.payload['confidence']
            # print('dm add({},{})'.format(input_iu.payload, confidence))

            # TODO: incremental
            # TODO: partially observed
            self._system.add_content('intent', ConceptVal('intent',act,confidence))

            for concept in concepts: 
                if '_confidence' in concept: continue
                conf = concepts['{}_confidence'.format(concept)]
                self._system.add_content('concept', ConceptVal(concept, str(concepts[concept]), conf))
            
            # if not self._system.is_paused:
            #     for concept in concepts:
            #         iuval = IUVal(str(concepts[concept]), concepts['{}_confidence'.format(concept)])
            #         self._cur_state.add_to_state(Assignment(concept, iuval))
            #     self._system.update() # update once after everything is in

        ## all other cases, treat the IU's payload as a dictionary and just put everything into the dialogue state
        state = input_iu.payload
        update_occured = False
        assert(type(state)==dict)
        for key in state:
            if self._variables is not None and key in self._variables:
                val = state[key]
                if isinstance(val, str) and val.isdigit():
                    val = float(val)
                if isinstance(val, int):
                    val = float(val)
                if key in self._prior_state:
                    if self._prior_state[key] == val: # no need to update
                        # print(self._prior_state)
                        pass
                #     else:
                #         print('dm state update {}={}'.format(key, val))
                # else:
                #     print('dm state update {}={}'.format(key, val))
                self._prior_state[key] = val
                self._system._cur_state.add_to_state(Assignment(key, val))
                update_occured = True
        if update_occured:
            self._system.update() # update opendial after the full state has been inserted

        return None

    def process_revoke(self, revoked_iu):
        pass #for now
        #print('dm revoke({})'.format(revoked_iu.payload))


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
            output_iu = self.create_iu(self._input_iu)
            output_iu.set_act(action, {})
            self.append(output_iu)
            # concept_val = state.query_prob('concept').get_best()
            # if isinstance(concept_val, ConceptVal):
            #     output_iu = self.create_iu(self._input_iu)
            #     output_iu.set_act(action, {concept_val._concept:concept_val._value}, concept_val._confidence)
            #     self.append(output_iu)


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
