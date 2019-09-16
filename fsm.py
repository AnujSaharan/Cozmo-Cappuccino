import sys
import cozmo
import datetime
import time
import numpy as np
import re
import joblib
from cozmo_classifier import ImageClassifier as cozmo_classifier
from idle_state import Idle_State as idle_state
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color

def finite_state_machine(state):
    states={
    # "drone": drone_state.execute(),
    # "inspection": inspection_state.execute(),
    # "order",
    # "plane",
    # "truck",
    # "hands",
    # "place",
    "none": cozmo.run_program(idle_state.execute)
    }
    return states.get(state, cozmo.run_program(idle_state.execute))

if __name__ == '__main__':
    cozmo.setup_basic_logging()
    try:
        cozmo.run_program(finite_state_machine("none"))
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)

