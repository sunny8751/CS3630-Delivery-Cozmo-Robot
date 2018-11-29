#!/usr/bin/env python

##############
#### Your name: Sunwoo Yim
##############

import numpy as np
import re
from sklearn import svm, metrics, model_selection
from skimage import io, feature, filters, exposure, color

class ImageClassifier:
    
    def __init__(self):
        self.classifer = None

    def imread_convert(self, f):
        return io.imread(f).astype(np.uint8)

    def load_data_from_folder(self, dir):
        # read all images into an image collection
        ic = io.ImageCollection(dir+"*.bmp", load_func=self.imread_convert)
        
        #create one large array of image data
        data = io.concatenate_images(ic)
        
        #extract labels from image names
        labels = np.array(ic.files)
        for i, f in enumerate(labels):
            m = re.search("_", f)
            labels[i] = f[len(dir):m.start()]
        
        return(data,labels)

    def extract_image_features(self, data):
        # Please do not modify the header above

        # extract feature vector from image data

        ########################
        ######## YOUR CODE HERE
        ########################
        feature_data = []
        for image in data:
            greyscale = color.rgb2grey(image)
            # apply gaussian filter first
            gaussian = filters.gaussian(greyscale)
            # edge detection
            prewitt = filters.prewitt(gaussian)

            hog = feature.hog(prewitt, orientations=17, pixels_per_cell=(24,24),
                cells_per_block=(8,8), block_norm='L2-Hys')
            feature_data.append(hog)

        
        # Please do not modify the return type below
        return(feature_data)

    def train_classifier(self, train_data, train_labels):
        # Please do not modify the header above
        
        # train model and save the trained model to self.classifier
        
        ########################
        ######## YOUR CODE HERE
        ########################
        lsvc = svm.SVC(kernel='linear')
        parameters = {'C': [1, 10, 100, 1000], 'gamma': [0.001, 0.0001], 'kernel': ['linear']}
        self.classifier = model_selection.GridSearchCV(lsvc, parameters)
        self.classifier.fit(train_data, train_labels)

    def predict_labels(self, data):
        # Please do not modify the header

        # predict labels of test data using trained model in self.classifier
        # the code below expects output to be stored in predicted_labels
        
        ########################
        ######## YOUR CODE HERE
        ########################
        predicted_labels = self.classifier.predict(data)
        
        # Please do not modify the return type below
        return predicted_labels
