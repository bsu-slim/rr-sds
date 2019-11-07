import sys
sys.path.append("/home/casey/git/rasa_nlu")

from rasa.nlu.training_data import load_data
from rasa.nlu.model import Trainer, Interpreter
from rasa.nlu import config
import multiprocessing
import time

confidence_threshold = 0.5
allowable_idle_seconds = 5
current_thread = None
frame_queue = []
frame = {}

dir = 'data/cozmo/nlu/'

def train_interpreter():
    training_data = load_data(dir) # loads all .md files
    trainer = Trainer(config.load("nlu_config.yml"))
    interpreter = trainer.train(training_data)
    test_interpreter_dir = trainer.persist(dir + '/models/')
    return test_interpreter_dir

# train
int_dir = train_interpreter()
print(int_dir)

