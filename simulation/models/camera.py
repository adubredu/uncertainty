
import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import time
import pybullet_data
from PIL import Image

direct = p.connect(p.GUI)  #, options="--window_backend=2 --render_device=0")
#egl = p.loadPlugin("eglRendererPlugin")

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF('plane.urdf')
tableid = p.loadURDF('table/table.urdf',[0,0,0],p.getQuaternionFromEuler([0,0,0]))
p.setAdditionalSearchPath('/home/developer/uncertainty/simulation/models')
pepsi = p.loadURDF('pepsi/pepsi.urdf',[0,0,0.65],p.getQuaternionFromEuler([1.57,0,0]))

width = 128
height = 128

fov = 60
aspect = width / height
near = 0.02
far = 2

# view_matrix = p.computeViewMatrix([0, 0, 3], [0, 0, 0], [1, 0, 0])
# projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

# Get depth values using the OpenGL renderer
# images = p.getCameraImage(width,
#                           height,
#                           view_matrix,
#                           projection_matrix,
#                           shadow=True,
#                           renderer=p.ER_BULLET_HARDWARE_OPENGL)
# # img_array = misc.imread(images[2])
# im = Image.fromarray(images[2])
# im.save('bottle.png')
# depth_buffer_opengl = np.reshape(images[3], [width, height])
# depth_opengl = far * near / (far - (far - near) * depth_buffer_opengl)
# seg_opengl = np.reshape(images[4], [width, height]) * 1. / 255.
# time.sleep(1)



while 1:
  view_matrix = p.computeViewMatrix([0, 0, 2], [0, 0, 0], [0, 1, 0])
  projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

  # Get depth values using the OpenGL renderer
  images = p.getCameraImage(width,
                            height,
                            view_matrix,
                            projection_matrix,
                            shadow=True,
                            renderer=p.ER_BULLET_HARDWARE_OPENGL)
  # img_array = misc.imread(images[2])
  im = Image.fromarray(images[2])
  im.save('pepsi.png')

  p.stepSimulation()
  time.sleep(2)