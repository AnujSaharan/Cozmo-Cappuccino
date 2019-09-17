# Cozmo Classifier is a script to extract image features and then 
# classify images using the pre-trained image_classifier model.

# CS 3630 - Fall 2019
# Georgia Institute of Technology
# Authors: Anuj Saharan, Riya Agrawal

import numpy as np
import re
from joblib import dump
from sklearn import svm, metrics
from skimage import io, feature, filters, exposure, color

class ImageClassifier:
    
    def __init__(self):
        self.classifier = None
    
    def extract_image_features(self, data):
        feature_data = []
        for pixel in data:
            feature_data.append(feature.hog(pixel, orientations=9, pixels_per_cell=(32, 32), cells_per_block=(4, 3)))        
        return(feature_data)
  
def main():
    img_clf = ImageClassifier()

if __name__ == "__main__":
    main()