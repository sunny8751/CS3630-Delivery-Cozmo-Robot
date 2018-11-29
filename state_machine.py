'''
Finite State Machine that encodes the
robot's states and behavior

authors: Sunwoo Yim, Hemanth Chittanuru
'''

import sys
import cozmo
import datetime
import time
import numpy as np
from imgclassification import ImageClassifier

robot = None
img_clf = ImageClassifier()

def train():
    # load images
    (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    
    # convert images into features
    train_data = img_clf.extract_image_features(train_raw)
    
    # train model and test on training data
    img_clf.train_classifier(train_data, train_labels)

def run(sdk_conn):
    global robot

    robot = sdk_conn.wait_for_robot()
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()

    robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    robot.say_text("before training").wait_for_completed()
    train()
    robot.say_text("after training").wait_for_completed()
    
    idle()

def getLabel():
    global robot
    MIN_HEAD_ANGLE = 0#-25
    MAX_HEAD_ANGLE = 20#44.5
    data_raw = []
    angle = MIN_HEAD_ANGLE
    for i in range(10):
        # tilt
        robot.set_head_angle(cozmo.util.degrees(angle)).wait_for_completed()
        #take image
        latest_image = robot.world.latest_image
        if not latest_image: continue
        new_image = latest_image.raw_image
        if not new_image: continue

        timestamp = datetime.datetime.now().strftime("%dT%H%M%S%f")
        new_image.save("./idle_imgs/" + timestamp + ".bmp")

        data_raw.append(np.array(new_image))
        angle += (MAX_HEAD_ANGLE - MIN_HEAD_ANGLE)/10
      
    # convert images into features
    data = img_clf.extract_image_features(data_raw)
        
    predicted_labels = img_clf.predict_labels(data)
    print(predicted_labels)
    
    # get most frequent
    frequency = {}
    for p_label in predicted_labels:
        if p_label == 'none': continue
        if p_label in frequency:
            frequency[p_label] += 1
        else:
            frequency[p_label] = 1
    
    label = 'none'
    maxFrequency = 0
    for l in frequency:
        if frequency[l] > maxFrequency:
            label = l
            maxFrequency = frequency[l]
    return label


if __name__ == '__main__':
    cozmo.setup_basic_logging()

    try:
        cozmo.connect(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)