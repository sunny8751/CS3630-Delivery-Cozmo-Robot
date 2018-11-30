import sys
import cozmo
import datetime
import time
import numpy as np
from imgclassification import ImageClassifier
import rrt
SCALE = 25

# marker id: type (string)
markersMap = {
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

cubeDropoffLocation = {
	"L1": (2,9.5,180),
	"U1": (14.5,16,90),
	"R1": (24,12.5,0),
	"R2": (24,6.5,0),
	"D1": (10.5,2,270),
	"D2": (18.5,2,270)
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

async def getMarkerLocations(robot, img_clf):
	for marker in markersLocationMap:
		# get marker label location
		location = markersLocationMap[marker]

		# go to markerLocation via RRT
		await pathPlan(robot, location)

		# identify the marker image
		markersMap[getLabel(robot, img_clf)] = marker
	return markersMap


async def goToCubes(robot, markersMap):

	locations = ["L1", "U1", "D2", "D1"]
	dests = ["drone", "plane", "", "inspector"]
	angle = 270

	for i, location in enumerate(locations):
		await pathPlan(robot, location)
		cozmo.turn_in_place(cozmo.util.degrees(angle))
		cube = robot.world.wait_for_observed_light_cube(timeout=5)
		if cube == None:
			continue
		print("Cozmo found a cube, and will now attempt to pick it up it:")
		pickupCube = robot.pickup_object(cube, True, False, num_retries=1)
		pickupCube.wait_for_completed()
		destination = markersMap[dests[i]]
		await pathPlan(robot, cubeDropoffLocation[destination])
		robot.place_object_on_ground_here(cube, False, 2).wait_for_completed()







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