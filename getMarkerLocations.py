import sys
import cozmo
import datetime
import time
import numpy as np
from imgclassification import ImageClassifier

# marker id: type (string)
markersMap = {
	"L1": None,
	"U1": None,
	"R1": None,
	"R2": None,
	"D1": None,
	"D2": None
}

# marker id: location (Pose)
markersLocationMap = {
	"L1": (0,9.5),
	"U1": (14.5,18),
	"R1": (26,12.5),
	"R2": (26,6.5),
	"D1": (10.5,0),
	"D2": (18.5,0)
}

# marker id: location robot should be in to detect marker
markersImageClassificationLocationMap = {
	"L1": (5,9.5,180),
	"U1": (14.5,13,90),
	"R1": (21,12.5,0),
	"R2": (21,6.5,0),
	"D1": (10.5,5,270),
	"D2": (18.5,5,270)
}

def getMarkerLocations(robot, img_clf):
    for marker in markersLocationMap:
    	markerLocation = markersLocationMap[marker]
    	# go to markerLocation via RRT
		markersMap[marker] = getLabel(robot, img_clf)
	return markersMap


def getLabel(robot, img_clf):
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