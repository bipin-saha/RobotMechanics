from visualize import *
from Transformation import *
import numpy as np

if __name__ == "__main__":
    transformation = translation(0, 0.1, 0)
    transformation = transformation + rpy2transformation(np.pi/3,0,0)
    plotTransformation(transformation)