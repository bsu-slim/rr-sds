# code for WAC classifiers

import numpy as np
from sklearn import linear_model
import pickle
from operator import itemgetter
import os
from sklearn.base import clone
from keras.models import Sequential
from keras.layers import Dense
from keras.layers import Dropout
from keras.callbacks import EarlyStopping
from tqdm import tqdm
from operator import itemgetter 
from sklearn.model_selection import train_test_split
import random
import logging

layer = 'probs'
name = 'efficientnet'
spec = '5k'

# suppress tensorflow warnings
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'  # error
logging.getLogger('tensorflow').setLevel(logging.ERROR)

NODE_DTYPE = np.dtype({
    'names': ['left_child', 'right_child', 'feature', 'threshold', 'impurity',
              'n_node_samples', 'weighted_n_node_samples'],
    'formats': [np.intp, np.intp, np.intp, np.float64, np.float64, np.intp,
                np.float64]
})

class WAC:
    
    def __init__(self,wac_dir,classifier_spec=(linear_model.LogisticRegression,{'penalty':'l2'}),compose_method='prod'):
        '''
        wac_dir: name of model for persistance and loading
        compose_method: prod, avg, sum
        
        to train:
        add observations, then call train() then persist()
        
        to evaluate:
        load() model, then call add_increment for each word in an utterance. Call new_utt() to start a new utterance. 
        '''
        self.wac = {}
        self.trained_wac = {}
        self.model_name=wac_dir
        self.current_utt = {}
        self.utt_words = []
        self.compose_method = compose_method
        self.classifier_spec = classifier_spec
        
    def vocab(self):
        return self.trained_wac.keys()
    
    def add_observation(self, word, features, label):
        if word not in self.wac: self.wac[word] = list()
        self.wac[word].append((features, label))
    
    def add_multiple_observations(self, word, features, labels):
        for f,p in zip(features,labels):
            self.add_observation(word, f, p)

    def create_negatives(self, num_negs=3):
        if len(self.wac) <= 1: return # need at least two words to find negs
        for word in self.wac:
            negs = []
            max_len = len(self.wac[word]) * num_negs
            while len(negs) < max_len:
                for iword in self.wac:
                    if iword == word: continue
                    i = random.sample(self.wac[iword],1)[0]
                    negs.append(i[0])
            for i in negs:
                self.add_observation(word, i, 0)

    def train(self, min_obs=4):
        classifier, classf_params = self.classifier_spec
        nwac = {}
        if len(self.wac) <= 1: return
        for word in self.wac:
            if len(self.wac[word]) < min_obs: continue
            this_classf = classifier(**classf_params)
            X,y = zip(*self.wac[word])
            this_classf.fit(X,y)
            nwac[word] = this_classf
        self.trained_wac = nwac
    
    def load_model(self):
        self.trained_wac = {}
        existing = [f.split('.')[0] for f in os.listdir(self.model_name)]
        print('loading WAC model')
        for item in tqdm(existing):
            with open('{}/{}.pkl'.format(self.model_name, item), 'rb') as handle:
                self.trained_wac[item] = pickle.load(handle)

    def persist_model(self):
        if len(self.trained_wac) == 0: return
        for word in self.trained_wac:
            with open('{}.pkl'.format(word), 'wb') as handle:
                pickle.dump(self.trained_wac[word],handle, protocol=pickle.HIGHEST_PROTOCOL)
 
    def get_current_prediction_state(self):
        return self.current_utt
    
    def get_predicted_intent(self):
        return max(self.get_current_prediction_state(), key=itemgetter(1))

    def add_increment(self, word, context):
        predictions  = self.proba(word, context)
        self.utt_words.append(word)
        return self.compose(predictions)

    def best_word(self, context):
        if len(self.trained_wac) > 0:
            probs = [(word, self.proba(word, context)) for word in self.trained_wac]
            res = max(probs, key = itemgetter(1))
            return res
        return None

    def best_object(self, word, context):
        preds =  self.proba(word, context)
        if preds is None: return None
        return max(preds, key=itemgetter(1))

    def proba(self, word, context):
        intents,feats = context
        if word not in self.trained_wac: return None # todo: return a distribution of all zeros?
        predictions = list(zip(intents,self.trained_wac[word].predict_proba(np.array(feats))[:,1]))
        return predictions
    
    def new_utt(self):
        self.current_utt = {} 
        self.utt_words = []
    
