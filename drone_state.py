import sys
import cozmo
import datetime
import time
import numpy as np
import re
import joblib
from cozmo_classifier import ImageClassifier as cozmo_classifier
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color

class Drone_State:
    def execute(robot : cozmo.robot.Robot):
        robot.say_text('Drone State').wait_for_completed()
        # cozmo.run_program(fsm.finite_state_machine("none"))