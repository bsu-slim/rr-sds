import functools
import threading
import time
import asyncio
import websocket
import json
import sys
import os
from collections import deque
import numpy as np

try:
    import thread
except ImportError:
    import _thread as thread

# retico
from retico.core import abstract
from retico.core.robot.common import RobotStateIU


class MistyStateModule(abstract.AbstractProducingModule):

    subscribe_tof = {
    "Operation": "subscribe",
    "Type": "TimeOfFlight",
    "DebounceMs": 100,
        "EventName": "TimeOfFlight",
    }

    subscribe_ss = {
    "Operation": "subscribe",
    "Type": "SelfState",
    "DebounceMs": 100,
        "EventName": "SelfState",
    }

    subscribe_ws = {
    "Operation": "subscribe",
    "Type": "WorldState",
    "DebounceMs": 100,
        "EventName": "WorldState",
    }

    @staticmethod
    def name():
        return "Misty II State Module"

    @staticmethod
    def description():
        return "A Module that tracks the Misty II Robot states"

    @staticmethod
    def input_ius():
        return [DialogueDecisionIU]

    @staticmethod
    def output_iu():
        return RobotStateIU

    def __init__(self, ip, subscribe_self=True,subscribe_tof=True,subscribe_world=True **kwargs):
        super().__init__(**kwargs)
        self.ip = ip
        self.state_queue = deque()
        self.sub_world = subscribe_world
        self.sub_tof = subscribe_tof
        self.sub_self = subscribe_self
        

    def process_iu(self, input_iu):
        if len(self.state_queue) > 0:
            state = self.state_queue.popleft()
            output_iu = self.create_iu(input_iu)
            output_iu.set_state(state)
            return output_iu

        return None
        
        
    def setup(self):
        
        def on_message(ws, message):
            self.state_queue.append(message)

        def on_error(ws, error):
            print(error)

        def on_close(ws):
            print("### misty socket closed ###")

        def on_open(ws):
            if self.sub_self: ws.send(json.dumps(subscribe_ss))
            if self.sub_tof: ws.send(json.dumps(subscribe_tof))
            if self.sub_world: ws.send(json.dumps(subscribe_ws)) 

        # websocket.enableTrace(True)
        ws = websocket.WebSocketApp("ws://{}pubsub".format(self.ip),
                                on_message = on_message,
                                on_error = on_error,
                                on_close = on_close)
        ws.on_open = on_open
        ws.run_forever()       