import sys
import cozmo
import datetime
import time
import numpy as np
import re
import joblib
from cozmo_classifier import ImageClassifier as cozmo_classifier
# from fsm import Finite_State_Machine as fsm
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color

class Idle_State:
    def execute(robot : cozmo.robot.Robot):
        robot.camera.image_stream_enabled = True
        robot.camera.color_image_enabled = False
        robot.camera.enable_auto_exposure()
        robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
        image_classifier = joblib.load('image_classifier.joblib')
    
        while(True):
            latest_image = robot.world.latest_image
            new_image = latest_image.raw_image

            data = np.array(new_image)
            data = np.asarray(data.astype(np.uint8))
            data = np.expand_dims(data, axis=0)

            data = cozmo_classifier.extract_image_features(0, data)
            label = image_classifier.predict(data);
            print(label)
            robot.say_text(str(label)).wait_for_completed()
            
            if(label != 'none'):
                fsm.finite_state_machine(label)