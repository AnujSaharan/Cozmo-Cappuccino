import sys
import cozmo
from cozmo.util import distance_mm, speed_mmps, degrees
import datetime
import time
import numpy as np
import re
import joblib
import asyncio
from cozmo_classifier import ImageClassifier as cozmo_classifier
# from states import States as states
# from idle_state import Idle_State as idle_state
# from drone_state import Drone_State as drone_state
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color

def idle_state(robot : cozmo.robot.Robot = None):
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    image_classifier = joblib.load('image_classifier.joblib')

    while(True):
        time.sleep(.3)
        latest_image = robot.world.latest_image
        new_image = latest_image.raw_image

        data = np.array(new_image)
        data = np.asarray(data.astype(np.uint8))
        data = np.expand_dims(data, axis=0)

        data = cozmo_classifier.extract_image_features(0, data)
        label = image_classifier.predict(data);
        print(label)
        # robot.say_text(str(label)).wait_for_completed()
        
        if label == 'drone':
            print('drone')
            robot.say_text('Drone State').wait_for_completed()
            lookaround = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
            cubes = robot.world.wait_until_observe_num_objects(num=1, object_type=cozmo.objects.LightCube, timeout=60)
            lookaround.stop()
            robot.pickup_object(cubes[0], num_retries=3).wait_for_completed()
            robot.drive_straight(distance_mm(100.0), speed_mmps(100.0)).wait_for_completed()
            robot.drive_straight(distance_mm(-100.0), speed_mmps(100.0)).wait_for_completed()
            robot.place_object_on_ground_here(cubes[0], num_retries=3).wait_for_completed()
            robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
            continue
            

        elif label == 'inspection':
            print('inspection')
            robot.say_text('Inspection State').wait_for_completed()
            for x in range(4):
                action1 = robot.turn_in_place(degrees(90)).wait_for_completed()
                action2 = robot.drive_straight(distance_mm(200.0), speed_mmps(100.0)).wait_for_completed()
                # action3 = robot.set_lift_height(1.0, duration=3.0, in_parallel=True).wait_for_completed()
                # action3 = robot.set_lift_height(0.0, duration=3.0).wait_for_completed()
            # robot.drive_straight(distance_mm(200.0), speed_mmps(100.0), in_parallel=True)
            # robot.set_lift_height(1.0, in_parallel=True, duration=3.0).wait_for_completed()
            robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
            continue
        elif label == 'order':
            print('order')
            robot.say_text('Order State').wait_for_completed()
            robot.drive_wheels(50.0, 120.0, duration=8.0)
            robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
            continue

        elif label == 'plane':
            robot.say_text('Plane State').wait_for_completed()
            continue
        elif label == 'truck':
            robot.say_text('Truck State').wait_for_completed()
            continue
        elif label == 'hands':
            robot.say_text('Hands State').wait_for_completed()
            continue
        elif label == 'place':
            robot.say_text('Place State').wait_for_completed()
            continue

        elif label == 'none':
            continue
            
# def drone_state(robot : cozmo.robot.Robot):
#     # robot.say_text('Drone State').wait_for_completed()
#     cozmo.run_program(finite_state_machine("none"))
#     # cozmo.run_program(fsm.finite_state_machine("none"))

# def finite_state_machine(state):
    # if state == 'drone':
        # cozmo.run_program(drone_state)
    # else if state == 'inspection':
        # cozmo.run_program(inspection_state.execute)
    # else if state == 'order':
    #     cozmo.run_program(order_state.execute)
    # else if state == 'plane':
    #     cozmo.run_program(plane_state.execute)
    # else if state == 'truck':
    #     cozmo.run_program(truck_state.execute)    
    # else if state == 'hands':
    #     cozmo.run_progra  m(hands_state.execute)
    # else if state == 'place':
    #     cozmo.run_program(place_state.execute)
    # elif state == 'none':
        # cozmo.run_program(idle_state)
        
def main():
    try:
        cozmo.run_program(idle_state).wait_for_completed()
    finally:
        print("Cozmo has stopped")
        

if __name__ ==  '__main__':
    main()