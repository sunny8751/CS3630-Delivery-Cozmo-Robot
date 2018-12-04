## If you run into an "[NSApplication _setup] unrecognized selector" problem on macOS,
## try uncommenting the following snippet
# Authors: Hemanth Chittanuru and Sunwoo Yim
# External Resources: Cozmo API

try:
    import matplotlib
    matplotlib.use('TkAgg')
except ImportError:
    pass

from skimage import color
import cozmo
import numpy as np
from numpy.linalg import inv
import threading
import time
import sys
import asyncio
from PIL import Image

from markers import detect, annotator

from grid import CozGrid
# from gui import GUIWindow
from particle import Particle, Robot
from setting import *
from particle_filter import *
from utils import *

#particle filter functionality
class ParticleFilter:

    def __init__(self, grid):
        self.particles = Particle.create_random(PARTICLE_COUNT, grid)
        self.grid = grid

    def update(self, odom, r_marker_list):

        # ---------- Motion model update ----------
        self.particles = motion_update(self.particles, odom)

        # ---------- Sensor (markers) model update ----------
        self.particles = measurement_update(self.particles, r_marker_list, self.grid)

        # ---------- Show current state ----------
        # Try to find current best estimate for display
        m_x, m_y, m_h, m_confident = compute_mean_pose(self.particles)
        return (m_x, m_y, m_h, m_confident)

# tmp cache
last_pose = cozmo.util.Pose(0,0,0,angle_z=cozmo.util.Angle(degrees=0))
flag_odom_init = False
picTime = time.time()

# goal location for the robot to drive to, (x, y, theta)
goal = (6,10,0)

# map
Map_filename = "map_arena.json"
grid = CozGrid(Map_filename)
# gui = GUIWindow(grid, show_camera=True)
pf = ParticleFilter(grid)

def compute_odometry(curr_pose, cvt_inch=True):
    '''
    Compute the odometry given the current pose of the robot (use robot.pose)

    Input:
        - curr_pose: a cozmo.robot.Pose representing the robot's current location
        - cvt_inch: converts the odometry into grid units
    Returns:
        - 3-tuple (dx, dy, dh) representing the odometry
    '''

    global last_pose, flag_odom_init
    last_x, last_y, last_h = last_pose.position.x, last_pose.position.y, \
        last_pose.rotation.angle_z.degrees
    curr_x, curr_y, curr_h = curr_pose.position.x, curr_pose.position.y, \
        curr_pose.rotation.angle_z.degrees

    dx, dy = rotate_point(curr_x-last_x, curr_y-last_y, -last_h)
    if cvt_inch:
        dx, dy = dx / grid.scale, dy / grid.scale

    return (dx, dy, diff_heading_deg(curr_h, last_h))


async def marker_processing(robot, camera_settings, show_diagnostic_image=False):
    '''
    Obtain the visible markers from the current frame from Cozmo's camera.
    Since this is an async function, it must be called using await, for example:

        markers, camera_image = await marker_processing(robot, camera_settings, show_diagnostic_image=False)

    Input:
        - robot: cozmo.robot.Robot object
        - camera_settings: 3x3 matrix representing the camera calibration settings
        - show_diagnostic_image: if True, shows what the marker detector sees after processing
    Returns:
        - a list of detected markers, each being a 3-tuple (rx, ry, rh)
          (as expected by the particle filter's measurement update)
        - a PIL Image of what Cozmo's camera sees with marker annotations
    '''

    global grid, picTime

    # Wait for the latest image from Cozmo
    picTime = time.time()
    image_event = await robot.world.wait_for(cozmo.camera.EvtNewRawCameraImage, timeout=30)

    # Convert the image to grayscale
    image = np.array(image_event.image)
    image = color.rgb2gray(image)

    # Detect the markers
    markers, diag = detect.detect_markers(image, camera_settings, include_diagnostics=True)

    # Measured marker list for the particle filter, scaled by the grid scale
    marker_list = [marker['xyh'] for marker in markers]
    marker_list = [(x/grid.scale, y/grid.scale, h) for x,y,h in marker_list]

    # Annotate the camera image with the markers
    if not show_diagnostic_image:
        annotated_image = image_event.image.resize((image.shape[1] * 2, image.shape[0] * 2))
        annotator.annotate_markers(annotated_image, markers, scale=2)
    else:
        diag_image = color.gray2rgb(diag['filtered_image'])
        diag_image = Image.fromarray(np.uint8(diag_image * 255)).resize((image.shape[1] * 2, image.shape[0] * 2))
        annotator.annotate_markers(diag_image, markers, scale=2)
        annotated_image = diag_image

    return marker_list, annotated_image

async def kidnapped(robot, pf, grid):
    robot.stop_all_motors()
    await robot.play_anim_trigger(cozmo.anim.Triggers.CodeLabUnhappy).wait_for_completed()



async def localize(robot: cozmo.robot.Robot):

    global flag_odom_init, last_pose
    global grid, pf
    global picTime

    # start streaming
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()

    # Obtain the camera intrinsics matrix
    fx, fy = robot.camera.config.focal_length.x_y
    cx, cy = robot.camera.config.center.x_y
    camera_settings = np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1]
    ], dtype=np.float)

    ###################

    # YOUR CODE HERE

    ###################

    localized = False
    curr_action = None
    while True:
        # time.sleep(0.05)
        # time.sleep(.5)
        if robot.is_picked_up:
            print("robot was picked up0")
            await kidnapped(robot, pf, grid)
            pf = ParticleFilter(grid)
            localized = False
            curr_action = None
            continue

        # if curr_action and type(curr_action)==cozmo.robot.TurnInPlace and curr_action.is_running: continue
        odom = compute_odometry(robot.pose)
        last_pose = robot.pose
        markers, camera_image = await marker_processing(robot, camera_settings)
        # gui.show_camera_image(camera_image)
        curr_loc = pf.update(odom, markers)
        isMeanGood = curr_loc[3]
        # print(isMeanGood)
        # gui.show_particles(pf.particles)
        # gui.show_mean(curr_loc[0], curr_loc[1], curr_loc[2], curr_loc[3])
        # gui.updated.set()

        if not localized:
            if isMeanGood:
                print("localized!")
                robot.stop_all_motors()
                localized = True
            else:
                # actively look around
                if not curr_action:
                    lookAround(robot, markers)
                    curr_action = True
                # curr_action = True
        if localized:
            #backtrack cuz of lag
            # totalTime = time.time()-picTime
            # await robot.turn_in_place(cozmo.util.degrees(180)).wait_for_completed()
            # robot.drive_wheel_motors(0,40)
            # timeDelta = time.time()
            # while (time.time() - timeDelta) < totalTime:
            #     continue
            # robot.stop_all_motors()
            # await robot.turn_in_place(cozmo.util.degrees(180)).wait_for_completed()

            curr_action = None
            x,y,h = compute_mean_pose(pf.particles)[:3]
            x *= 25
            y *= 25
            h -= 45
            print("before turn to 0:", x,y,h)
            await robot.turn_in_place(cozmo.util.degrees(-h)).wait_for_completed()
            h = 0
            print("after turn to 0:", x,y,h)

            # move forward while inside center obstacle
            dist = 0
            while x >= 245 and x <= 405 and y >= 145 and y <= 305:
                dist += 10
                x += 10 * np.cos(np.deg2rad(h))
                y += 10 * np.sin(np.deg2rad(h))
            await robot.drive_straight(cozmo.util.distance_mm(dist), speed=cozmo.util.speed_mmps(60)).wait_for_completed()

            print("localize xyh:",x,y,h)

            pf = ParticleFilter(grid)
            localized = False
            curr_action = None
            # newAngle = diff_heading_deg(h, robot.pose.rotation.angle_z.degrees) % 360
            # return (x-robot.pose.position.x,y-robot.pose.position.y, newAngle)
            return [x,y,h]

def lookAround(robot, markers):
    print("looking around...")
    from random import randint
    from cozmo.util import degrees, Pose
    # x = randint(-100,100)
    # y = randint(-100,100)
    # return robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(0)))
    # return robot.drive_wheel_motors(randint(5, 30), randint(5, 30))
    return robot.drive_wheel_motors(40,0)
    # if len(markers) == 0:
    #     # robot.set_head_angle(degrees(randint(0,10)), in_parallel=True)
    #     return robot.turn_in_place(angle=cozmo.util.degrees(-30))#, in_parallel=True)
    # else:
    #     return robot.drive_straight(distance=cozmo.util.distance_mm(40), speed=cozmo.util.speed_mmps(50))


async def driveToGoal(robot, robot_pose):
    print("driving to goal...")
    from cozmo.util import degrees, Pose
    global goal
    import math
    robot.stop_all_motors()

    robot_x, robot_y, robot_h = robot_pose
    goal_x, goal_y, goal_h = goal
    # await robot.turn_in_place(degrees(-robot_h)).wait_for_completed()
    # print("turned first")
    # if robot.is_picked_up:
    #     return False
    dx, dy = goal_x - robot_x, goal_y - robot_y
    turn_degree = (np.arctan2(dy,dx) * 180 / np.pi) % 360
    # turn_degree -= 70
    # turn_degree %= 360
    # goal_pose = Pose(dx*24,dy*24,0,angle_z=degrees(turn_degree))
    # action = robot.go_to_pose(goal_pose, relative_to_robot=True)
    await robot.turn_in_place(degrees(-robot_h+turn_degree)).wait_for_completed()
    print("turned first")
    if robot.is_picked_up:
        return False

    dist = math.sqrt((dx*24)**2+(dy*24)**2)
    dist_so_far = 0
    while (dist_so_far < dist):
        d = 25
        d = min(d,dist-dist_so_far)
        await robot.drive_straight(d, speed=40).wait_for_completed()
        if robot.is_picked_up:
            return False
        dist_so_far += 25
    print("went to pose")
    await robot.turn_in_place(degrees(-turn_degree)).wait_for_completed()
    if robot.is_picked_up:
        return False
    print("turned second")
    return True



class CozmoThread(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        global goal
        goal_x, goal_y, goal_h = goal
        print(goal_x*grid.scale, goal_y*grid.scale)
        cozmo.robot.Robot.drive_off_charger_on_connect = False  # Cozmo can stay on his charger
        cozmo.run_program(run, use_viewer=False)


if __name__ == '__main__':

    # cozmo thread
    cozmo_thread = CozmoThread()
    cozmo_thread.start()

    # init
    gui.show_particles(pf.particles)
    gui.show_mean(0, 0, 0)
    gui.start()

