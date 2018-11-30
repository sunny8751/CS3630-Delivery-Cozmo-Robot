import threading

from gui import Visualizer
from train_classifier import createImageClassifier
import cozmo
import numpy as np
import sys
from getMarkerLocations import getMarkerLocations, goToCubes
from cmap import *
from rrt import node_generator
# import pickle
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

for map in [cubeDropoffLocation, cornerLocations]:
    for key,value in map.items():
        if len(value) == 3:
            map[key] = (value[0]*scale, value[1]*scale, value[2])
        else:
            map[key] = (value[0]*scale, value[1]*scale)

cmap = CozMap("emptygrid.json", node_generator)

async def run(robot):
	global cubeDropoffLocation
	# train images to create image classifier
	# filename = 'lab2classifier.sav'
	# img_clf = imgclassification.ImageClassifier()
	# img_clf.classifier = pickle.load(open(filename, 'rb'))

	img_clf = createImageClassifier()



	# localize
	start_x,start_y,start_h = await localize(robot)
	start_h -= robot.pose.rotation.angle_z.degrees

	startPosition = (start_x, start_y, start_h)
	#
	# # add grey square into path planning- DONE IN RRT.PY
	#
	# # store image marker locations
	markersMap = await getMarkerLocations(robot, img_clf, startPosition, cmap)
	#
	await goToCubes(robot, markersMap, startPosition, cmap)
	# '''
	# for each cube:
	# 	path plan to cube location
	# 	if cube exists:
	# 		lift cube
	# 		path plan to corresponding image marker
	# 		drop cube at location
	# '''

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
	global cmap, stopevent
	stopevent = threading.Event()

	robot_thread = RobotThread()
	robot_thread.start()


	visualizer = Visualizer(cmap)
	visualizer.start()
	stopevent.set()

	# robot = None
	# run(robot)