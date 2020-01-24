
import queue
import threading
from retico.core import abstract
from retico.core.text.common import SpeechRecognitionIU
from retico.core.audio.common import AudioIU
import time

try:
    import azure.cognitiveservices.speech as speechsdk
except ImportError:
    print("""
    Importing the Speech SDK for Python failed.
    use the following pip command: 'pip install azure-cognitiveservices-speech'
    Make sure you're using at least Python version 3.5 or later.""")
    exit(1)


class AzureASRModule(abstract.AbstractModule):
    """A Module that recognizes speech by utilizing the Azure Speech API."""

    def __init__(self, speech_key, language="en-US",
                service_region = "westus",
                endpoint = "https://westus.api.cognitive.microsoft.com/",
                 **kwargs):
        """Initialize the GoogleASRModule with the given arguments.

        Args:
            language (str): The language code the recognizer should use.
            nchunks (int): Number of chunks that should trigger a new
                prediction.
            rate (int): The framerate of the input audio
        """
        super().__init__(**kwargs)
        self.speech_key = speech_key        # This is the key for SlimASRComp
        self.service_region = service_region                     # SlimASRComp is configured for western us
        self.endpoint = endpoint # Endpoint for the resource, optional
        self.language = language 
        self.audio_buffer = queue.Queue()
        self.done = False
        self.latest_input_iu = None
        self.asr_stream = None  
        self.speech_recognizer = None                                    # Configured to American English.

    @staticmethod
    def name():
        return "Azure ASR Module"

    @staticmethod
    def description():
        return "A Module that incrementally recognizes speech."

    @staticmethod
    def input_ius():
        return [AudioIU]

    @staticmethod
    def output_iu():
        return SpeechRecognitionIU

    def process_iu(self, input_iu):
        # print('before', self.audio_buffer.qsize())
        self.audio_buffer.put(input_iu.raw_audio)
        # print('after', self.audio_buffer.qsize())
        if not self.latest_input_iu:
            self.latest_input_iu = input_iu
        return None


    def result_callback(self, event_type, evt):
        """callback to display a result to the console. I have it printing to the console
        for the time being, but I can easily configure it to return a dictionary or any other
        sort of information if needed. Each print out to the console would be an entry in a dictionary
        or JSON. I currently have this set to print out for both 'recognizing'
        and 'recognized' signals, something that can also change easily."""
        # print("{}: {}\n".format(event_type, evt.result.text))
        text = evt.result.text
        output_iu = self.create_iu(self.latest_input_iu)
        self.latest_input_iu = None
        output_iu.set_asr_results([text], text, 0, 1.0, False)
        if event_type == 'RECOGNIZED':
            output_iu.committed = True
        self.append(output_iu)
        

    def stop_callback(self, event_type, evt):
        """callback that stops continuous recognition upon receiving an event. This is called when
        the session gets stopped or there is an error thrown. Cancel signals are the only way errors
        are thrown, but a cancel signal will also happen if the reading is coming from an audio file 
        and it reached the end.
        
        Session_stopped signals seem to be thrown by doing something like editing the file while the
        recognizer is running, along with running the 'speech_recognizer.stop_continuous_recognition()'
        method."""
        print('{} on {}'.format(event_type, evt))
        self.speech_recognizer.stop_continuous_recognition()
        self.done = True

    def setup(self):
        """gives an example how to use a push audio stream to recognize speech from a custom audio
        source"""

        CHUNKSIZE = 1024
        SAMPLE_WIDTH = 2
        RATE = 16000
        CHANNELS = 1

        speech_config = speechsdk.SpeechConfig(subscription=self.speech_key, region=self.service_region, speech_recognition_language=self.language)

        # setup the audio stream
        self.asr_stream = speechsdk.audio.PushAudioInputStream()
        audio_config = speechsdk.audio.AudioConfig(stream=self.asr_stream)

        # instantiate the speech recognizer with push stream input
        self.speech_recognizer = speechsdk.SpeechRecognizer(speech_config=speech_config, audio_config=audio_config)


        # Connect callbacks to the events fired by the speech recognizer
        self.speech_recognizer.recognizing.connect(lambda evt: self.result_callback('RECOGNIZING', evt))
        self.speech_recognizer.recognized.connect(lambda evt: self.result_callback('RECOGNIZED', evt))
        self.speech_recognizer.session_started.connect(lambda evt: print('SESSION STARTED: {}'.format(evt)))
        self.speech_recognizer.session_stopped.connect(lambda evt: self.stop_callback('SESSION STOPPED', evt))
        self.speech_recognizer.canceled.connect(lambda evt: self.stop_callback('CANCELED', evt))
        self.speech_recognizer.start_continuous_recognition()
        t = threading.Thread(target=self._generator)
        t.start()



    def _generator(self):
        try:
            while not self.done:
                frames = self.audio_buffer.get()
                if not frames:
                    break
                self.asr_stream.write(frames)
                time.sleep(.1)
        finally:
            # stop recognition and clean up
            self.asr_stream.close()
            self.speech_recognizer.stop_continuous_recognition()


    def shutdown(self):
        self.audio_buffer.put(None)
