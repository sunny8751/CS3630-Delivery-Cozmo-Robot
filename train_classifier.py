import sys
import cozmo
import datetime
import time
import numpy as np
from imgclassification import ImageClassifier

img_clf = ImageClassifier()

def train():
    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    
    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)
    
    # train model and test on training data
    img_clf.train_classifier(train_data, train_labels)

def createImageClassifier():
	train()
	return img_clf