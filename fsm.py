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

def idle_state(robot : cozmo.robot.Robot):
    
    robot.camera.image_stream_enabled = True
    robot.camera.color_image_enabled = False
    robot.camera.enable_auto_exposure()
    robot.set_head_angle(cozmo.util.degrees(0)).wait_for_completed()
    image_classifier = joblib.load('image_classifier.joblib')

    isClassifying = 1
    while isClassifying:
        time.sleep(.2)
        latest_image = robot.world.latest_image
        new_image = latest_image.raw_image

        data = np.array(new_image)
        data = np.asarray(data.astype(np.uint8))
        data = np.expand_dims(data, axis=0)

        data = cozmo_classifier.extract_image_features(0, data)
        label = image_classifier.predict(data);
        print(label)
        robot.say_text(str(label)).wait_for_completed()


if __name__ == '__main__':
    cozmo.setup_basic_logging()

    try:
        cozmo.connect(idle_state)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
