# retico
from retico.core import abstract
from retico.core.dialogue.common import DialogueActIU

# zeromq & supporting libraries
import zmq, json
import datetime


class ReaderSingleton:
    __instance = None
    @staticmethod 
    def getInstance():
        """ Static access method. """
        if ReaderSingleton.__instance == None:
            ReaderSingleton()
        return ReaderSingleton.__instance
    def __init__(self, ip, port):
        """ Virtually private constructor. """
        if ReaderSingleton.__instance == None:
            self.socket = zmq.Context().socket(zmq.SUB)
            self.socket.connect("tcp://{}:{}".format(ip, port))
            ReaderSingleton.__instance = self

class WriterSingleton:
    __instance = None
    @staticmethod 
    def getInstance():
        """ Static access method. """
        if WriterSingleton.__instance == None:
            WriterSingleton()
        return WriterSingleton.__instance
    def __init__(self, ip, port):
        """ Virtually private constructor. """
        if WriterSingleton.__instance == None:
            context = zmq.Context()
            self.socket = context.socket(zmq.PUB)
            self.socket.bind('tcp://{}:{}'.format(ip, port))
            WriterSingleton.__instance = self

class ZeroMQIU(abstract.IncrementalUnit):

    @staticmethod
    def type():
        return "ZeroMQ Incremental Unit"

    def __init__(self, creator=None, iuid=0, previous_iu=None, grounded_in=None,
                 payload=None, **kwargs):
        """Initialize the DialogueActIU with act and concepts.

        Args:
            act (string): A representation of the act.
            concepts (dict): A representation of the concepts as a dictionary.
        """
        super().__init__(creator=creator, iuid=iuid, previous_iu=previous_iu,
                         grounded_in=grounded_in, payload=payload)

    def set_payload(self, payload):
        self.payload = payload
        



class ZeroMQReader(abstract.AbstractProducingModule):

    """A ZeroMQ Reader Module

    Attributes:
        
    """
    @staticmethod
    def name():
        return "ZeroMQ Reader Module"

    @staticmethod
    def description():
        return "A Module providing reading from a ZeroMQ bus"

    @staticmethod
    def output_iu():
        return ZeroMQIU 

    def __init__(self, topic,  **kwargs):
        """Initializes the ZeroMQReader.

        Args: topic(str): the topic/scope where the information will be read.
            
        """
        super().__init__(**kwargs)
        self.topic = topic
        self.reader = None

    def process_iu(self, input_iu):
        '''
        This assumes that the message is json formatted, then packages it as payload into an IU
        '''
        [topic, message] = self.reader.recv_multipart()
        j = json.loads(message)
        output_iu = self.create_iu()
        output_iu.set_payload(j)

        return output_iu


    def prepare_run(self):
        self.reader = ReaderSingleton.getInstance().socket
        self.reader.setsockopt(zmq.SUBSCRIBE, self.topic)

    def setup(self):
        pass
        

class ZeroMQWriter(abstract.AbstractModule):

    """A ZeroMQ Writer Module

    Attributes:
        
    """
    @staticmethod
    def name():
        return "ZeroMQ Writer Module"

    @staticmethod
    def description():
        return "A Module providing writing onto a ZeroMQ bus"

    @staticmethod
    def output_iu():
        return None 

    @staticmethod
    def input_ius():
        return [abstract.IncrementalUnit] 

    def __init__(self, topic,  **kwargs):
        """Initializes the ZeroMQReader.

        Args: topic(str): the topic/scope where the information will be read.
            
        """
        super().__init__(**kwargs)
        self.topic = topic
        self.writer = None

    def process_iu(self, input_iu):
        '''
        This assumes that the message is json formatted, then packages it as payload into an IU
        '''
        payload = {}
        payload['message'] = json.dumps(input_iu.payload)
        payload['originatingTime'] = datetime.datetime.now().isoformat()
        self.writer.send_multipart([self.topic.encode(), json.dumps(payload).encode('utf-8')])

    def setup(self):
        self.writer = WriterSingleton.getInstance().socket