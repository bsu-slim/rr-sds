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
from retico.core.dialogue.common import DialogueDecisionIU


class MistyStateModule(abstract.AbstractProducingModule):

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

    def subscribe_msg(self, item):
        return  {
        "Operation": "subscribe",
        "Type": 'ActuatorPosition',
        "DebounceMs": 100,
        "EventName": item,
        "EventConditions": [
         {
            "Property": "sensorName",
            "Inequality": "==",
            "Value": item
         }
        ]
        }

    def __init__(self, ip, subscribe_to=['Actuator_HeadPitch',
                                         'Actuator_HeadYaw',
                                         'Actuator_HeadRoll',
                                         'Actuator_LeftArm',
                                         'Actuator_RightArm'], **kwargs):
        super().__init__(**kwargs)
        self.ip = ip
        self.subscribe_to = subscribe_to
        self.state_queue = deque()

    def process_iu(self, input_iu):
        if len(self.state_queue) > 0:
            state = json.loads(self.state_queue.popleft())
            message = state['message']
            if 'value' in message:
                state = {state['eventName']:message['value']}
                output_iu = self.create_iu(input_iu)
                output_iu.set_state(state)
                return output_iu

        return None
        

    def run_websocket(self):
        def on_message(ws, message):
            self.state_queue.append(message)

        def on_error(ws, error):
            print(error)

        def on_close(ws):
            print("### misty socket closed ###")

        def on_open(ws):
            for item in self.subscribe_to:
                ws.send(json.dumps(self.subscribe_msg(item)))

        ws = websocket.WebSocketApp("ws://{}/pubsub".format(self.ip),
                                on_message = on_message,
                                on_error = on_error,
                                on_close = on_close)
        ws.on_open = on_open
        ws.run_forever()   
        
    def setup(self):
        t = threading.Thread(target=self.run_websocket)
        t.start()    