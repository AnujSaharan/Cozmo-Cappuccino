# Finite State Machine is a script to model certain actions as states
# and then run them on the Cozmo.

# CS 3630 - Fall 2019
# Georgia Institute of Technology
# Authors: Anuj Saharan, Riya Agrawal

import sys
import cozmo
from cozmo.util import distance_mm, speed_mmps, degrees
import datetime
import time
import numpy as np
import re
import joblib
from cozmo_classifier import ImageClassifier as cozmo_classifier
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color

def idle_state(robot : cozmo.robot.Robot = None):
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    image_classifier = joblib.load('image_classifier.joblib')

    while(True):
        robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
        robot.set_lift_height(0.0, duration=0.5).wait_for_completed()
        
        states = []
        for counter in range(4):
            time.sleep(0.1)
            latest_image = robot.world.latest_image
            new_image = latest_image.raw_image
    
            data = np.array(new_image)
            data = np.asarray(data.astype(np.uint8))
            data = np.expand_dims(data, axis=0)
            
            data = cozmo_classifier.extract_image_features(0, data)
            current_state = image_classifier.predict(data);
            
            states = states + [current_state]
        
        sliding_window_one = (states[-1])
        sliding_window_two = (states[-2])
        sliding_window_three = (states[-3])
        
        if(sliding_window_one == sliding_window_two):
            state = sliding_window_one
        elif(sliding_window_two == sliding_window_three):
            state = sliding_window_two
        elif(sliding_window_one == sliding_window_three):
            state = sliding_window_one
        else:
            state = sliding_window_one
        
        print(state)
        if state == 'drone':
            print('Commencing Drone Actions')
            robot.say_text('Drone', duration_scalar=0.7).wait_for_completed()
            
            lookaround = robot.start_behavior(cozmo.behavior.BehaviorTypes.LookAroundInPlace)
            
            cubes = robot.world.wait_until_observe_num_objects(num=1, object_type=cozmo.objects.LightCube, timeout=60)
            lookaround.stop()
            
            robot.pickup_object(cubes[0], num_retries=3).wait_for_completed()
            
            robot.drive_straight(distance_mm(100.0), speed_mmps(500.0)).wait_for_completed()
            
            robot.place_object_on_ground_here(cubes[0], num_retries=3).wait_for_completed()
            
            robot.drive_straight(distance_mm(-100.0), speed_mmps(500.0)).wait_for_completed()
            
            continue
            
        elif state == 'inspection':
            print('Commencing Inspection Actions')
            robot.say_text('Inspection', duration_scalar=0.7).wait_for_completed()
            
            for counter in range(4):
                action1 = robot.drive_straight(distance_mm(200), speed_mmps(40), in_parallel=True)
                
                action2 = robot.set_lift_height(1.0, in_parallel=True, duration=3.0)
                action2.wait_for_completed()
                
                action2 = robot.set_lift_height(0.0, in_parallel=True, duration=3.0)
                action2.wait_for_completed()

                action1.wait_for_completed()
                
                robot.turn_in_place(degrees(90)).wait_for_completed()
            continue
        
        elif state == 'order':
            print('Commencing Order Actions')
            robot.say_text('Order', duration_scalar=0.7).wait_for_completed()
            
            robot.drive_wheels(50.0, 120.0, duration=8.2)
            continue

        # elif state == 'plane':
        #     robot.say_text('Plane State').wait_for_completed()
        #     continue
        # 
        # elif state == 'truck':
        #     robot.say_text('Truck State').wait_for_completed()
        #     continue
        # 
        # elif state == 'hands':
        #     robot.say_text('Hands State').wait_for_completed()
        #     continue
        # 
        # elif state == 'place':
        #     robot.say_text('Place State').wait_for_completed()
        #     continue
        # 
        elif state == 'none':
            continue
            
def main():
    try:
        cozmo.run_program(idle_state).wait_for_completed()
    finally:
        print("The Finite State Machine has stopped.")
        
if __name__ ==  '__main__':
    main()