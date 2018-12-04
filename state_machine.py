import threading

import time

from gui import Visualizer
from train_classifier import createImageClassifier
import cozmo
import numpy as np
import sys
from getMarkerLocations import getMarkerLocations, goToCubes
from cmap import *
from rrt import node_generator, pathPlan
import pickle
import imgclassification

from go_to_goal import localize

cubeDropoffLocation = {
	"L1": (2,9.5,180),
	"U1": (14.5,16,90),
	"R1": (24,12.5,0),
	"R2": (24,6.5,0),
	"D1": (10.5,2,270),
	"D2": (18.5,2,270)
}

cornerLocations = {
	"A": (0,18),
	"B": (26,18),
	"C": (0,0),
	"D": (26,18)
}

scale = 25

testMarkersMap = {
	"plane": "U1",
	"order": "D1",
	"drone" : "R2",
	"hands": "R1",
	"place": "L1",
	"inspection": "D2"
}

for map in [cubeDropoffLocation, cornerLocations]:
    for key,value in map.items():
        if len(value) == 3:
            map[key] = (value[0]*scale, value[1]*scale, value[2])
        else:
            map[key] = (value[0]*scale, value[1]*scale)

cmap = CozMap("emptygrid.json", node_generator)

async def run(robot):
	from cozmo.util import distance_mm, speed_mmps, degrees
	# print(robot.pose.position.x, robot.pose.position.y, robot.pose.rotation.angle_z.degrees)
	# await robot.drive_straight(distance_mm(400), speed_mmps(80), should_play_anim=False).wait_for_completed()
	# await robot.turn_in_place(cozmo.util.degrees(-90)).wait_for_completed()
	# print(robot.pose.position.x, robot.pose.position.y, robot.pose.rotation.angle_z.degrees)
	# await robot.turn_in_place(cozmo.util.degrees(-90)).wait_for_completed()
	# print(robot.pose.position.x, robot.pose.position.y, robot.pose.rotation.angle_z.degrees)
	# await robot.drive_straight(distance_mm(100), speed_mmps(80), should_play_anim=False).wait_for_completed()
	# print(robot.pose.position.x, robot.pose.position.y, robot.pose.rotation.angle_z.degrees)
	
	global cubeDropoffLocation
	startTime = time.time()
	# train images to create image classifier
	# filename = 'lab2classifier.sav'
	# img_clf = imgclassification.ImageClassifier()
	# img_clf.classifier = pickle.load(open(filename, 'rb'))

	img_clf = imgclassification.ImageClassifier()
	# filename = 'lab6classifier.sav'
	filename = 'lab2classifier.sav'
	img_clf.classifer = pickle.load(open(filename, 'rb'))


	# localize
	# better word for start position is "position delta", difference of actual location and robot's pose

	shouldLocalize = True

	if shouldLocalize:
		robot_pose = await localize(robot)

		await pathPlan(robot, (150,250), robot_pose, cmap)
		await robot.turn_in_place(degrees(-robot_pose[2])).wait_for_completed()

		await robot.say_text("Default position").wait_for_completed()

	# robot_pose = [152.4, 254, 0]
	robot_pose = [150, 250, 0]


	# start_x = 20 * 25 - robot.pose.position.x
	# start_y = 8 * 25 - robot.pose.position.y
	# start_h = -robot.pose.rotation.angle_z.degrees

	# startPosition = (start_x, start_y, start_h)

	# print("start position: ", startPosition)
	# print("robot pose: ", robot.pose.position.x, robot.pose.position.y, robot.pose.rotation.angle_z.degrees)
	# print("curr position: ", startPosition[0]+robot.pose.position.x, startPosition[1]+robot.pose.position.y, robot.pose.rotation.angle_z.degrees + startPosition[2])
	# print("localization time: ", time.time()-startTime)

	# store image marker locations
	cmap.set_start(Node((robot_pose[0], robot_pose[1])))
	#markersMap = await getMarkerLocations(robot, img_clf, robot_pose, cmap)

	if shouldLocalize:
		robot_pose = await localize(robot)

		await pathPlan(robot, (150,250), robot_pose, cmap)
		await robot.turn_in_place(degrees(-robot_pose[2])).wait_for_completed()

		await robot.say_text("Default position").wait_for_completed()

		await goToCubes(robot, testMarkersMap, robot_pose, cmap)



class RobotThread(threading.Thread):
	"""Thread to run cozmo code separate from main thread
	"""

	def __init__(self):
		threading.Thread.__init__(self, daemon=True)

	def run(self):
		# Please refrain from enabling use_viewer since it uses tk, which must be in main thread
		cozmo.run_program(run,use_3d_viewer=False, use_viewer=False)
		stopevent.set()

if __name__ == '__main__':
	# global cmap, 
	global stopevent
	stopevent = threading.Event()

	robot_thread = RobotThread()
	robot_thread.start()


	visualizer = Visualizer(cmap)
	visualizer.start()
	stopevent.set()

	# robot = None
	# run(robot)