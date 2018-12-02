import sys
from cozmo import util, setup_basic_logging, ConnectionError, connect, robot
import datetime
import time
import numpy as np
import re
from skimage import io, feature, filters, color
from sklearn import svm
import pickle

#Rahul Ajmera, Prabhath Chandanala

class CozmoState:
    IDLE = 0
    DRONE = 1
    ORDER = 2
    INSPECTION = 3

class ImageClassifier:

    def __init__(self):
        self.classifer = None

    def imread_convert(self, f):
        return io.imread(f).astype(np.uint8)

    def load_data_from_folder(self, dir):
        # read all images into an image collection
        ic = io.ImageCollection(dir + "*.bmp", load_func=self.imread_convert)

        # create one large array of image data
        data = io.concatenate_images(ic)

        # extract labels from image names
        labels = np.array(ic.files)
        for i, f in enumerate(labels):
            m = re.search("_", f)
            labels[i] = f[len(dir):m.start()]

        return (data, labels)

    def predictOneImage(self, data):
        gray_image = color.rgb2gray(data)
        im_gray = filters.gaussian(gray_image, sigma=0.4)
        features = feature.hog(im_gray, orientations=10, pixels_per_cell=(48, 48), cells_per_block=(4, 4),
                               feature_vector=True, block_norm='L2-Hys')
        return (features)

    def extract_image_features(self, data):
        feature_data = []
        for image in data:
            gray_image = color.rgb2gray(image)
            im_gray = filters.gaussian(gray_image, sigma=0.4)
            features = feature.hog(im_gray, orientations=10, pixels_per_cell=(48, 48), cells_per_block=(4, 4),
                                   feature_vector=True, block_norm='L2-Hys')
            # feature_data = feature.hog(data)
            feature_data.append(features)
            # Please do not modify the return type below
        return (feature_data)

    def train_classifier(self, train_data, train_labels):

        # Please do not modify the header above

        # train model and save the trained model to self.classifier
        self.classifer = svm.LinearSVC()
        self.classifer.fit(train_data, train_labels)

    def predict_labels(self, data):
        predicted_labels = self.classifer.predict(data)

        # Please do not modify the return type below
        return predicted_labels

    def getOneLabel(self, data):
        #
        #
        #
        #
        #
        #TODO: Line below (reshape) might need to be commented out. Not sure?

        data = data.reshape(1,-1)
        predicted_labels = self.classifer.predict(data)

        # Please do not modify the return type below
        return predicted_labels


def run(sdk_conn):
    filename = 'lab2classifier.sav'
    robot = sdk_conn.wait_for_robot()
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()

    robot.set_head_angle(util.degrees(-5.0)).wait_for_completed()

    state = CozmoState.IDLE

    img_clf = ImageClassifier()

    # (train_raw, train_labels) = img_clf.load_data_from_folder('./train/')
    # (test_raw, test_labels) = img_clf.load_data_from_folder('./test/')
    #
    # # convert images into features
    # train_data = img_clf.extract_image_features(train_raw)
    # test_data = img_clf.extract_image_features(test_raw)
    #
    # # train model and test on training data
    # img_clf.train_classifier(train_data, train_labels)

    img_clf.classifer = pickle.load(open(filename, 'rb'))


    while (state != None):
        if (state == CozmoState.IDLE):
            next_state = classifyImage(robot, img_clf)
            if (next_state == 'drone'):
                robot.say_text(next_state).wait_for_completed()
                state = CozmoState.DRONE
            elif (next_state == 'order'):
                robot.say_text(next_state).wait_for_completed()
                state = CozmoState.ORDER
            elif (next_state == 'inspection'):
                robot.say_text(next_state).wait_for_completed()
                state = CozmoState.INSPECTION

        elif (state == CozmoState.DRONE):
            pickup_cube(robot)
            state = CozmoState.IDLE
        elif (state == CozmoState.ORDER):
            #Drive robot in a circle
            robot.drive_wheels(1000, 100, None, None, 10)
            state = CozmoState.IDLE
        elif (state == CozmoState.INSPECTION):
            # Cozmo goes in a 20cm/side square while raising and lowering lift
            inspection_state(robot)
            state = CozmoState.IDLE



def pickup_cube(robot: robot.Robot):
    robot.set_head_angle(util.degrees(-5.0)).wait_for_completed()

    print("Cozmo is waiting until he sees a cube")
    cube = robot.world.wait_for_observed_light_cube()

    print("Cozmo found a cube, and will now attempt to pick it up it:")

    pickupCube = robot.pickup_object(cube, True, False, num_retries=1)
    pickupCube.wait_for_completed()

    driveForward = robot.drive_straight(util.distance_mm(100), util.speed_mmps(100))
    driveForward.wait_for_completed()
    placeCube = robot.place_object_on_ground_here(cube, False, 2)
    placeCube.wait_for_completed()
    driveBackward = robot.drive_straight(util.distance_mm(-100), util.speed_mmps(100))
    driveBackward.wait_for_completed()


def inspection_state(robot: robot.Robot):
    isLifted = False
    for _ in range(4):
        action1 = robot.drive_straight(util.distance_mm(200), util.speed_mmps(100))
        # robot.drive_straight(util.distance_mm(200), util.speed_mmps(100))
        if not isLifted:
            # robot.set_lift_height(1.0, 20.0, 20.0, 3.0, True, 2)
            action2 = robot.set_lift_height(1.0, 20.0, 20.0, 3.0, True, 2)
            isLifted = True
        else:
            # robot.set_lift_height(0.0, 20.0, 20.0, 3.0, True, 2)
            action2 = robot.set_lift_height(0.0, 20.0, 20.0, 3.0, True, 2)
            isLifted = False
        action1.wait_for_completed()
        action2.wait_for_completed()

        robot.turn_in_place(util.degrees(90)).wait_for_completed()
    #
    #     #Sets Lift back to 0 height and state = idle.

    robot.set_lift_height(0.0, 20.0, 20.0, 3.0, True, 2)


def classifyImage(robot: robot.Robot, img_clf: ImageClassifier):
    time.sleep(.5)
    latest_image = robot.world.latest_image
    new_image = latest_image.raw_image
    np_image = np.array(new_image)
    extracted_image = img_clf.predictOneImage(np_image)
    predicted_image = img_clf.getOneLabel(extracted_image)
    next_state = predicted_image[0]
    return next_state


if __name__ == '__main__':
    setup_basic_logging()

    try:
        connect(run)
    except ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)