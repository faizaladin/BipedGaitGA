import random
import mujoco
import mujoco_viewer
import numpy as np
import random
from scipy.optimize import minimize
from numpy.random import randint

model = mujoco.MjModel.from_xml_path('cassie.xml')
data = mujoco.MjData(model)
viewer = mujoco_viewer.MujocoViewer(model, data)

view.render()