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

def run(sdk_conn):
    
    robot = sdk_conn.wait_for_robot()
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

        # robot.say_text("Image taken").wait_for_completed()
        timestamp = datetime.datetime.now().strftime("%dT%H%M%S%f")
        # new_image.save("./imagestream/" + "test_"  + ".bmp")
         # data = io.imread('./imagestream/test_.bmp')
        data = np.array(new_image)
        data = np.asarray(data.astype(np.uint8))
        data = np.expand_dims(data, axis=0)
        # print(data.shape)
        data = cozmo_classifier.extract_image_features(0, data)
        print(image_classifier.predict(data))
        robot.say_text(str(image_classifier.predict(data))).wait_for_completed()


if __name__ == '__main__':
    cozmo.setup_basic_logging()

    try:
        cozmo.connect(run)
    except cozmo.ConnectionError as e:
        sys.exit("A connection error occurred: %s" % e)
