from train_classifier import createImageClassifier
import cozmo
import numpy as np
import sys
from getMarkerLocations import getMarkerLocations, goToCubes

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

def run(robot):
	global cubeDropoffLocation
	# train images to create image classifier
	img_clf = createImageClassifier()

	# localize
	localize(robot)

	# add grey square into path planning- DONE IN RRT.PY

	# store image marker locations
	markersMap = getMarkerLocations(robot, img_clf)

	goToCubes(robot, markersMap)
	'''
	for each cube:
		path plan to cube location
		if cube exists:
			lift cube
			path plan to corresponding image marker
			drop cube at location
	'''

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