import cozmo
import threading
import json
import math
import sys
import time
import numpy as np


async def CozmoPID(robot: cozmo.robot.Robot):
    global stopevent
    with open("./config.json") as file:
        config = json.load(file)
    kp = config["kp"]
    ki = config["ki"]
    kd = config["kd"]
    ###############################
    # PLEASE ENTER YOUR CODE BELOW
    await robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    # find cube
    cube_position_x = 0
    cube_position_y = 0

    time.sleep(1)
    for obj in robot.world.visible_objects:
        cube_position_x = obj.pose.position.x
        cube_position_y = obj.pose.position.y
        break

    # print(robot.pose.position.y)
    # print("X:{0} Y:{1}".format(cube_position_x, cube_position_y))

    # PID
    prev_x = 0
    prev_y = 0
    y_error_sum = 0
    previousTime = time.time()
    iteration_count = 0

    reference_x = 130
    reference_y = 0

    while True:
        if iteration_count > 1:
            time_elapsed = time.time() - previousTime
            previousTime = time.time()
        else:
            time_elapsed = 1

        x_distance_from_cube = (cube_position_x - robot.pose.position.x)
        # print("\nCube Position: {0}\nRobot Pose: {1}\nX Distance: {2}\n".format(
        # cube_position_x, robot.pose.position.x, x_distance_from_cube))
        y_distance_from_cube = (cube_position_y - robot.pose.position.y)

        if cube_position_x == 0:  # Too close to the cube
            reference_x = 20

        error_x = x_distance_from_cube - reference_x
        error_y = y_distance_from_cube - reference_y

        y_error_sum += error_y

        px = kp * error_x
        ix = ki * 0
        dx = kd * (error_x - prev_x) / time_elapsed
        pidx = px + ix + dx

        py = kp * error_y
        iy = ki * y_error_sum
        dy = kd * (error_y - prev_y) / time_elapsed
        pidy = py + iy + dy

        prev_x = error_x
        prev_y = error_y

        # print("ITERATION #{5}\nX PID: {0}\nY PID: {6}\n\nX from Cube: {1}\nY from Cube: {2}\n\nX Error: {3}\nY error: {4}\n\nTime Elapsed: {7}\nPx: {8}\nPy: {9}\nDx: {10}\nDy: {11}\n\n".format(
        # str(pidx), str(x_distance_from_cube), str(y_distance_from_cube), str(error_x), str(error_y), str(iteration_count), str(pidy), str(time_elapsed), str(px), str(py), str(dx), str(dy)))

        robot.drive_wheel_motors(pidx, pidx)
        iteration_count += 1

        await robot.wait_for(cozmo.robot.EvtRobotStateUpdated)


class RobotThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self, daemon=False)

    def run(self):
        cozmo.run_program(CozmoPID)
        stopevent.set()


if __name__ == "__main__":
    global stopevent
    stopevent = threading.Event()
    robot_thread = RobotThread()
    robot_thread.start()
    stopevent.set()
