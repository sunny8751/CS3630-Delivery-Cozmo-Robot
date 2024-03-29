try:
    import matplotlib
    matplotlib.use('TkAgg')
except ImportError:
    pass

import cozmo
import math
import sys
import time
import random
import numpy as np



from cmap import *
from gui import *
from utils import *

MAX_NODES = 20000


def step_from_to(node0, node1, limit=75):
    ########################################################################
    # TODO: please enter your code below.
    # 1. If distance between two nodes is less than limit, return node1
    # 2. Otherwise, return a node in the direction from node0 to node1 whose
    #    distance to node0 is limit. Recall that each iteration we can move
    #    limit units at most
    # 3. Hint: please consider using np.arctan2 function to get vector angle
    # 4. Note: remember always return a Node object
    dist = get_dist(node0, node1)
    if dist <= limit: return node1
    dx, dy = node1.x-node0.x, node1.y-node0.y
    angle = np.arctan2(dy,dx)
    x = node0.x + limit * np.cos(angle)
    y = node0.y + limit * np.sin(angle)

    return Node((x,y))
    ############################################################################


def node_generator(cmap):
    rand_node = None
    ############################################################################
    # TODO: please enter your code below.
    # 1. Use CozMap width and height to get a uniformly distributed random node
    # 2. Use CozMap.is_inbound and CozMap.is_inside_obstacles to determine the
    #    legitimacy of the random node.
    # 3. Note: remember always return a Node object
    ############################################################################
    prob = random.randint(0,100)
    if prob < 5:
      goal = cmap.get_goals()[random.randint(0,len(cmap.get_goals())) - 1]
      return Node([goal.x, goal.y])
    map_width, map_height = cmap.get_size()
    x = random.randint(0, map_width)
    y = random.randint(0, map_height)
    rand_node = Node([x,y])
    while not cmap.is_inbound(rand_node) and not cmap.is_inside_obstacles(rand_node):
        x = random.randint(0, map_width)
        y = random.randint(0, map_height)
        rand_node = Node([x,y])
    return rand_node


def RRT(cmap, start):
    cmap.add_node(start)
    map_width, map_height = cmap.get_size()
    while (cmap.get_num_nodes() < MAX_NODES):
        ########################################################################
        # TODO: please enter your code below.
        # 1. Use CozMap.get_random_valid_node() to get a random node. This
        #    function will internally call the node_generator above
        # 2. Get the nearest node to the random node from RRT
        # 3. Limit the distance RRT can move
        # 4. Add one path from nearest node to random node
        #
        rand_node = cmap.get_random_valid_node()
        cmap_nodes = cmap.get_nodes()

        nearest_node = None
        minDist = sys.maxsize
        for node in cmap_nodes:
            d = get_dist(node, rand_node)
            if d < minDist:
              minDist = d
              nearest_node = node

        rand_node = step_from_to(nearest_node, rand_node)
        pass
        ########################################################################
        time.sleep(0.01)
        cmap.add_path(nearest_node, rand_node)
        if cmap.is_solved():
            break

    path = cmap.get_path()
    smoothed_path = cmap.get_smooth_path()

    if cmap.is_solution_valid():
        print("A valid solution has been found :-) ")
        print("Nodes created: ", cmap.get_num_nodes())
        print("Path length: ", len(path))
        print("Smoothed path length: ", len(smoothed_path))
    else:
        print("Please try again :-(")


def get_global_node(local_angle, local_origin, node):
    """Helper function: Transform the node's position (x,y) from local coordinate frame specified by local_origin and local_angle to global coordinate frame.
                        This function is used in detect_cube_and_update_cmap()
        Arguments:
        local_angle, local_origin -- specify local coordinate frame's origin in global coordinate frame
        local_angle -- a single angle value
        local_origin -- a Node object

        Outputs:
        new_node -- a Node object that decribes the node's position in global coordinate frame
    """
    ########################################################################
    x = node.x
    y = node.y

    new_x = (x * math.cos(local_angle)) - (y * math.sin(local_angle)) + local_origin.x
    new_y = (y * math.cos(local_angle)) + (x * math.sin(local_angle)) + local_origin.y

    new_node = Node((new_x, new_y))

    return new_node


async def detect_cube_and_update_cmap(robot, marked, cozmo_pos, cmap):
    """Helper function used to detect obstacle cubes and the goal cube.
       1. When a valid goal cube is detected, old goals in cmap will be cleared and a new goal corresponding to the approach position of the cube will be added.
       2. Approach position is used because we don't want the robot to drive to the center position of the goal cube.
       3. The center position of the goal cube will be returned as goal_center.

        Arguments:
        robot -- provides the robot's pose in G_Robot
                 robot.pose is the robot's pose in the global coordinate frame that the robot initialized (G_Robot)
                 also provides light cubes
        cozmo_pose -- provides the robot's pose in G_Arena
                 cozmo_pose is the robot's pose in the global coordinate we created (G_Arena)
        marked -- a dictionary of detected and tracked cubes (goal cube not valid will not be added to this list)

        Outputs:
        update_cmap -- when a new obstacle or a new valid goal is detected, update_cmap will set to True
        goal_center -- when a new valid goal is added, the center of the goal cube will be returned
    """

    # Padding of objects and the robot for C-Space
    cube_padding = 60.
    cozmo_padding = 100.

    # Flags
    update_cmap = False
    goal_center = None

    # Time for the robot to detect visible cubes
    time.sleep(1)

    for obj in robot.world.visible_objects:

        if obj.object_id in marked:
            continue
        print("Saw obstacle")
        # Calculate the object pose in G_Arena
        # obj.pose is the object's pose in G_Robot
        # We need the object's pose in G_Arena (object_pos, object_angle)
        dx = obj.pose.position.x - robot.pose.position.x
        dy = obj.pose.position.y - robot.pose.position.y

        object_pos = Node((cozmo_pos.x+dx, cozmo_pos.y+dy))
        object_angle = obj.pose.rotation.angle_z.radians

        # The goal cube is defined as robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id
        '''
        if robot.world.light_cubes[cozmo.objects.LightCube1Id].object_id == obj.object_id:

            # Calculate the approach position of the object
            local_goal_pos = Node((0, -cozmo_padding))
            goal_pos = get_global_node(object_angle, object_pos, local_goal_pos)

            # Check whether this goal location is valid
            if cmap.is_inside_obstacles(goal_pos) or (not cmap.is_inbound(goal_pos)):
                print("The goal position is not valid. Please remove the goal cube and place in another position.")
            else:
                cmap.clear_goals()
                cmap.add_goal(goal_pos)
                goal_center = object_pos
        '''
        # Define an obstacle by its four corners in clockwise order
        obstacle_nodes = []
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, -cube_padding))))
        obstacle_nodes.append(get_global_node(object_angle, object_pos, Node((-cube_padding, cube_padding))))
        print([(node.x,node.y) for node in obstacle_nodes])
        cmap.add_obstacle(obstacle_nodes)
        marked[obj.object_id] = obj
        update_cmap = True

    return update_cmap, goal_center


class RobotThread(threading.Thread):
    """Thread to run cozmo code separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        # Please refrain from enabling use_viewer since it uses tk, which must be in main thread
        cozmo.run_program(CozmoPlanning,use_3d_viewer=False, use_viewer=False)
        stopevent.set()


class RRTThread(threading.Thread):
    """Thread to run RRT separate from main thread
    """

    def __init__(self):
        threading.Thread.__init__(self, daemon=True)

    def run(self):
        while not stopevent.is_set():
            RRT(cmap, cmap.get_start())
            time.sleep(100)
            cmap.reset()
        stopevent.set()

# async def findCornerCubes(robot, cmap):
#     # Allows access to map and stopevent, which can be used to see if the GUI
#     # has been closed by checking stopevent.is_set()
#     global stopevent
#     from cozmo.util import degrees, distance_mm, speed_mmps, Pose, Angle
#
#     ########################################################################
#     # TODO: please enter your code below.
#     # Description of function provided in instructions
#     await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
#
#     width, height = cmap.get_size()
#     # scale = 25
#
#     marked = {}
#     curr_pos = Node((robot.pose.position.x, robot.pose.position.y))
#     # print("start pose",x,y)
#     cmap.set_start(curr_pos)
#     update_goal = False
#
#     while True:
#         curr_pos = Node((robot.pose.position.x, robot.pose.position.y))
#         cmap.set_start(curr_pos)
#
#         update_goal, goal_center = await detect_cube_and_update_cmap(robot, marked, curr_pos)
#         print(update_goal,goal_center)
#         if update_goal:
#             cmap.reset()
#
#         if goal_center is not None:
#             print("Saw cube")
#             RRT(cmap, cmap.get_start())
#
#         # if cmap.is_solved():
#             # goToCubes()
#             # drive to this corner
#             # do stuff with the cube
#             # delete this goal
#             # maybe reset the cmap?


# start is a tuple (x,y) in mm
async def pathPlan(robot, goal, robot_pose, cmap):
    # cmap = CozMap("emptygrid.json", node_generator)
    cmap.reset()
    cmap.clear_goals()

    cmap.set_start(Node((robot_pose[0], robot_pose[1])))
    cmap.add_goal(Node(goal))

    # entered middle obstancle square in emptygrid.json

    RRT(cmap, cmap.get_start())
    if (cmap.is_solution_valid()):
        path = cmap.get_smooth_path()
        await driveAlongPath(robot, path, robot_pose, cmap)
    else:
        print("CANNOT RRT TO MARKER!!!")

async def driveAlongPath(robot, path, robot_pose, cmap):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global stopevent
    from cozmo.util import degrees, distance_mm, speed_mmps, Pose, Angle

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions

    width, height = cmap.get_size()

    marked = {}
    # prev_pos = path[0]
    # print("length to path is",len(path))
    redoRRT = True
    while redoRRT:
        print("Driving along path")
        redoRRT = False
        for node in path:

            # print(robot.pose.position.x, robot.pose.position.y)
            curr_pos = Node((robot_pose[0], robot_pose[1]))

            cmap.set_start(curr_pos)

            dx,dy = node.x-curr_pos.x, node.y-curr_pos.y
            targetAngle = np.degrees(np.arctan2(dy,dx))
            # print("Dx: " , dx)
            # print("Dy: " , dy)
            # print("angle: " , targetAngle)

            #dAngle = (angle-robot.pose.rotation.angle_z.degrees - startPosition[2])%360
            # if dAngle >= 180: dAngle = -(360-dAngle)
            dAngle = diff_heading_deg(targetAngle, robot_pose[2])
            # print("Distance: ", dist, "Dangle: ", dAngle)
            # prev_pose = robot.pose

            prev_pose = robot.pose
            # prev_pos = get_global_node(robot_pose[2], curr_pos, Node((0,0)))
            # print("prev", (prev_pos.x,prev_pos.y), "now", robot_pose)
            await robot.turn_in_place(degrees(dAngle)).wait_for_completed()

            # prev_pos = get_global_node(robot_pose[2], curr_pos, Node((0,0)))
            # print("prev", (prev_pos.x,prev_pos.y), "now", robot_pose)

            # now_pos = get_global_node(robot_pose[2]+dAngle, Node((0,0)), robot.pose.position)
            print("inaccuracies: ", robot.pose.position.x - prev_pose.position.x, robot.pose.position.y - prev_pose.position.y, abs(robot.pose.rotation.angle_z.degrees - prev_pose.rotation.angle_z.degrees) - abs(dAngle))
            # print("inaccuracies: ", now_pos.x - prev_pos.x, now_pos.y - prev_pos.y, abs(robot.pose.rotation.angle_z.degrees - prev_pose.rotation.angle_z.degrees) - abs(dAngle))
            # robot_pose[0] += robot.pose.position.x - prev_pose.position.x
            # robot_pose[1] += robot.pose.position.y - prev_pose.position.y
            # robot_pose[0] += now_pos.x - prev_pos.x
            # robot_pose[1] += now_pos.y - prev_pos.y
            # robot_pose[2] += robot.pose.rotation.angle_z.degrees - prev_pose.rotation.angle_z.degrees

            robot_pose[2] += dAngle

            dist = np.sqrt(dx ** 2 + dy ** 2)

            #time.sleep(1)
            await robot.drive_straight(distance_mm(dist), speed_mmps(40), should_play_anim=False).wait_for_completed()
            robot_pose[0] += dx
            robot_pose[1] += dy

            curr_pos = Node((robot_pose[0], robot_pose[1]))
            cmap.set_start(curr_pos)
            #time.sleep(1)
            #await robot.go_to_pose(cozmo.util.pose_z_angle(node.x, node.y, 0, angle_z = cozmo.util.Angle(dAngle))).wait_for_completed()
            # prev_pos = node

    curr_pos = Node((robot_pose[0], robot_pose[1]))
    cmap.set_start(curr_pos)

# start is a tuple (x,y) in mm
async def pathPlanCubes(robot, goal, robot_pose, cmap):
    # cmap = CozMap("emptygrid.json", node_generator)
    cmap.reset()
    cmap.clear_goals()

    cmap.set_start(Node((robot_pose[0], robot_pose[1])))
    cmap.add_goal(Node(goal))

    # entered middle obstancle square in emptygrid.json

    RRT(cmap, cmap.get_start())
    if (cmap.is_solution_valid()):
        path = cmap.get_smooth_path()
        return await driveAlongPathCubes(robot, path, robot_pose, cmap)
    else:
        print("CANNOT RRT TO MARKER!!!")


async def driveAlongPathCubes(robot, path, robot_pose, cmap):
    # Allows access to map and stopevent, which can be used to see if the GUI
    # has been closed by checking stopevent.is_set()
    global stopevent
    from cozmo.util import degrees, distance_mm, speed_mmps, Pose, Angle

    ########################################################################
    # TODO: please enter your code below.
    # Description of function provided in instructions

    width, height = cmap.get_size()

    marked = {}
    # prev_pos = path[0]
    # print("length to path is",len(path))
    redoRRT = True
    while redoRRT:
        print("Driving along path")
        redoRRT = False
        copyPath = list(path)

        curr_pos = Node((robot_pose[0], robot_pose[1]))
        cmap.set_start(curr_pos)

        for node in copyPath:

            # print(robot.pose.position.x, robot.pose.position.y)
            curr_pos = Node((robot_pose[0], robot_pose[1]))

            # TODO: FIX THIS SHIT
            update_cmap, goal_center = await detect_cube_and_update_cmap(robot, marked, curr_pos, cmap)
            if update_cmap:
                print("REDOING RRT BECAUSE IT SAW A CUBE")
                cmap.reset()
                cmap.set_start(curr_pos)
                RRT(cmap, curr_pos)
                # i = 0
                # while not cmap.is_solution_valid() and i < 20:
                #     cmap.reset()
                #     cmap.set_start(curr_pos)
                #     RRT(cmap, curr_pos)
                #     i += 1
                path = cmap.get_smooth_path()
                if cmap.is_solution_valid():
                    # path = cmap.get_smooth_path()
                    print("Found solution")
                else:
                    print("NO SOLUTION")
                redoRRT = True
                break

            cmap.set_start(curr_pos)

            dx,dy = node.x-curr_pos.x, node.y-curr_pos.y
            targetAngle = np.degrees(np.arctan2(dy,dx))
            # print("Dx: " , dx)
            # print("Dy: " , dy)
            # print("angle: " , targetAngle)

            #dAngle = (angle-robot.pose.rotation.angle_z.degrees - startPosition[2])%360
            # if dAngle >= 180: dAngle = -(360-dAngle)
            dAngle = diff_heading_deg(targetAngle, robot_pose[2])
            # print("Distance: ", dist, "Dangle: ", dAngle)
            # prev_pose = robot.pose

            prev_pose = robot.pose
            # prev_pos = get_global_node(robot_pose[2], curr_pos, Node((0,0)))
            # print("prev", (prev_pos.x,prev_pos.y), "now", robot_pose)
            await robot.turn_in_place(degrees(dAngle)).wait_for_completed()

            # prev_pos = get_global_node(robot_pose[2], curr_pos, Node((0,0)))
            # print("prev", (prev_pos.x,prev_pos.y), "now", robot_pose)

            # now_pos = get_global_node(robot_pose[2]+dAngle, Node((0,0)), robot.pose.position)
            print("inaccuracies: ", robot.pose.position.x - prev_pose.position.x, robot.pose.position.y - prev_pose.position.y, abs(robot.pose.rotation.angle_z.degrees - prev_pose.rotation.angle_z.degrees) - abs(dAngle))
            # print("inaccuracies: ", now_pos.x - prev_pos.x, now_pos.y - prev_pos.y, abs(robot.pose.rotation.angle_z.degrees - prev_pose.rotation.angle_z.degrees) - abs(dAngle))
            # robot_pose[0] += robot.pose.position.x - prev_pose.position.x
            # robot_pose[1] += robot.pose.position.y - prev_pose.position.y
            # robot_pose[0] += now_pos.x - prev_pos.x
            # robot_pose[1] += now_pos.y - prev_pos.y
            # robot_pose[2] += robot.pose.rotation.angle_z.degrees - prev_pose.rotation.angle_z.degrees

            robot_pose[2] += dAngle

            dist = np.sqrt(dx ** 2 + dy ** 2)

            #time.sleep(1)
            await robot.drive_straight(distance_mm(dist), speed_mmps(80), should_play_anim=False).wait_for_completed()
            robot_pose[0] += dx
            robot_pose[1] += dy

            curr_pos = Node((robot_pose[0], robot_pose[1]))
            cmap.set_start(curr_pos)
            #time.sleep(1)
            #await robot.go_to_pose(cozmo.util.pose_z_angle(node.x, node.y, 0, angle_z = cozmo.util.Angle(dAngle))).wait_for_completed()
            # prev_pos = node

    curr_pos = Node((robot_pose[0], robot_pose[1]))
    cmap.set_start(curr_pos)
    return robot.pose

if __name__ == '__main__':
    global cmap, stopevent
    stopevent = threading.Event()
    robotFlag = False
    for i in range(0,len(sys.argv)):
        if (sys.argv[i] == "-robot"):
            robotFlag = True
    if (robotFlag):
        cmap = CozMap("maps/emptygrid.json", node_generator)
        robot_thread = RobotThread()
        robot_thread.start()
    else:
        cmap = CozMap("emptygrid.json", node_generator)
        sim = RRTThread()
        sim.start()
    visualizer = Visualizer(cmap)
    visualizer.start()
    stopevent.set()