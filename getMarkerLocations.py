import sys
import cozmo
import datetime
import time
import numpy as np
from imgclassification import ImageClassifier
import rrt
import cmap
from utils import Node

SCALE = 25

# marker id: type (string)
markersMap = {
}

# marker id: location (Pose)
markersLocationMap = {
    "L1": (0, 9.5), # x+offset
    "U1": (14.5, 18), # y-offset
    "R1": (26, 12.5), # x-offset
    "R2": (26, 6.5), #x-offset
    "D1": (10.5, 0), # y+offset
    "D2": (18.5, 0) #y+offset
}

cubeDropoffLocation = {
    "L1": (2, 9.5, 180),
    "U1": (14.5, 16, 90),
    "R1": (24, 12.5, 0),
    "R2": (24, 6.5, 0),
    "D1": (10.5, 2, 270),
    "D2": (18.5, 2, 270)
}

# marker id: location robot should be in to detect marker
markersImageClassificationLocationMap = {
    "L1": (6, 9.5, 180),
    "U1": (14.5, 13, 90),
    "R1": (20, 12.5, 0),
    "R2": (20, 6.5, 0),
    "D1": (10.5, 5.5, 270),
    "D2": (18.5, 5.5, 270)
}

markersSet = {"drone", "plane", "hands", "place", "inspection", "order"}

scale = 25
for map in [cubeDropoffLocation, markersLocationMap, markersImageClassificationLocationMap]:
    for key,value in map.items():
        if len(value) == 3:
            map[key] = (value[0]*scale, value[1]*scale, value[2])
        else:
            map[key] = (value[0]*scale, value[1]*scale)

async def getMarkerLocations(robot, img_clf, startPosition, cmap):
    for marker in markersImageClassificationLocationMap:
        # get marker label location
        x,y,h = markersImageClassificationLocationMap[marker]

        # go to markerLocation via RRT
        await rrt.pathPlan(robot, (x,y), startPosition, cmap)
        dAngle = (h - robot.pose.rotation.angle_z.degrees - startPosition[2]) % 360
        if dAngle >= 180: dAngle = -(360 - dAngle)
        await robot.turn_in_place(cozmo.util.degrees(dAngle)).wait_for_completed()
        # TODO set heading angle
        print("Marker Path Planning complete " +  marker)

        # identify the marker image
        markersMap[await getLabel(robot, img_clf)] = marker

    return markersMap


async def goToCubes(robot, markersMap, startPosition, cmap):
    locations = ["L1", "U1", "D2", "D1"]
    dests = ["drone", "plane", "place", "inspection"]
    angle = 300
    #robot.set_heading


    cmap.set_start(Node((startPosition[0]+robot.pose.position.x, startPosition[1]+robot.pose.position.y)))

    for i, location in enumerate(locations):
        await rrt.pathPlan(robot, markersImageClassificationLocationMap[location], startPosition, cmap)
        x,y,h = markersImageClassificationLocationMap[location]
        dAngle = (h - robot.pose.rotation.angle_z.degrees - startPosition[2]) % 360
        if dAngle >= 180: dAngle = -(360 - dAngle)
        await robot.turn_in_place(cozmo.util.degrees(dAngle)).wait_for_completed()
        await robot.turn_in_place(cozmo.util.degrees(angle)).wait_for_completed()
        cube = await robot.world.wait_for_observed_light_cube(timeout=10)
        if cube == None:
            continue
        print("Cozmo found a cube, and will now attempt to pick it up it:")
        await robot.pickup_object(cube, True, False, num_retries=3).wait_for_completed()
        destination = markersMap[dests[i]]
        # start = robot.pose.position.x, robot.pose.position.y, -robot.pose.rotation.angle_z.degrees
        await rrt.pathPlan(robot, cubeDropoffLocation[destination], startPosition, cmap)
        await robot.place_object_on_ground_here(cube, False, 2).wait_for_completed()


async def getLabel(robot, img_clf):
    print("getting label")
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    MIN_HEAD_ANGLE = 8  # -25
    MAX_HEAD_ANGLE = 12  # 44.5
    data_raw = []
    angle = MIN_HEAD_ANGLE
    for i in range(5):
        # tilt
        await robot.set_head_angle(cozmo.util.degrees(angle)).wait_for_completed()
        # take image
        time.sleep(.5)
        latest_image = robot.world.latest_image
        if not latest_image: continue
        new_image = latest_image.raw_image
        if not new_image: continue

        timestamp = datetime.datetime.now().strftime("%dT%H%M%S%f")
        new_image.save("./test_imgs/" + timestamp + ".bmp")

        data_raw.append(np.array(new_image))
        angle += (MAX_HEAD_ANGLE - MIN_HEAD_ANGLE) / 5

    # convert images into features
    # print("raw data is", data_raw)
    data = img_clf.extract_image_features(data_raw)

    #extracted_image = img_clf.predictOneImage(data)
    predicted_labels = img_clf.predict_labels(data)
    print(predicted_labels)

    # get most frequent
    frequency = {}
    for p_label in predicted_labels:
        if p_label == 'none': continue
        if p_label not in markersSet: continue
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

    print(label)
    # if label in markersSet:
    #     markersSet.remove(label)
    # elif len(markersSet) == 1:
    #     label = list(markersSet)[0]
    # else:
    #     print("error: weird label")
    await robot.say_text(label).wait_for_completed()
    return label
