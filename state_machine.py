from train_classifier import createImageClassifier
import cozmo
import numpy as np
import sys

from go_to_goal import localize

def run(robot):
    # localize
    localize(robot)
    # add grey square into path planning

    # store image marker locations

    '''
    for each cube:
        path plan to cube location
        if cube exists:
            lift cube
            path plan to corresponding image marker
            drop cube at location
    '''


if __name__ == '__main__':

    # train images to create image classifier
    img_clf = createImageClassifier()

    robot = None
    run(robot)