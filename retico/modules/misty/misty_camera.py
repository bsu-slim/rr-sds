import functools
import threading
import time
import asyncio
import websocket
import json
import sys
import os
import numpy as np
import requests
from PIL import Image
from io import BytesIO
import base64
try:
    import thread
except ImportError:
    import _thread as thread

# retico
from retico.core import abstract
from retico.core.visual.common import ImageIU

class MistyCameraModule(abstract.AbstractProducingModule):
    @staticmethod
    def name():
        return "Misty II Camera Module"

    @staticmethod
    def description():
        return "A Module that tracks the Misty II Robot camera"

    @staticmethod
    def output_iu():
        return ImageIU

    def take_picture(self, ip):
        resp = requests.get('http://'+ip+'/api/cameras/rgb?base64=true&width='+str(self.width)+'&height='+str(self.height))
        resp = resp.json()
        return (resp['result'])

    def __init__(self, ip, width=320, height=427, **kwargs):
        super().__init__(**kwargs)
        self.width = width
        self.height = height
        self.ip = ip

    def process_iu(self, input_iu):
        result = self.take_picture(self.ip)
        im = Image.open(BytesIO(base64.b64decode(result.get('base64'))))
        output_iu = self.create_iu(input_iu)
        output_iu.set_image(im, 1, 1)
        return output_iu
