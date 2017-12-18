import os
import sys
import json
import urllib
import urllib2
import numpy as np
import h5py as h5
from scipy.misc import imread
import IPython
import math


object_name = '025_mug'
data_type = 'google_512k'
object_mass = 0.414

path = os.path.join(os.path.expanduser('~'), 'catkin_ws', 'src', 'urdf_models', 'urdf_models', 'models', '025_mug_google_512k')
urdf_file = os.path.join(path, 'urdf', '025_mug_google_512k.urdf')
 
print (urdf_file)

f = open(urdf_file, 'w')
urdf_str = """
<?xml version="1.0" ?> 
<robot name=\"""" + object_name + """\">
  <link name=\"""" + object_name + """_link">
    <inertial>
      <origin xyz="0 0 0" />
      <mass value=\"""" + str(object_mass) + """\" />
      <inertia  ixx=\"1.0\" ixy=\"0.0\"  ixz=\"0.0\"  iyy=\"1.0\"  iyz=\"0.0\"  izz=\"1.0\" />
    </inertial>
    <visual>
      <origin xyz=\"0 0 0\"/>
      <geometry>
        <mesh filename=\"file:///""" + path + """/""" + 'mesh' + """/nontextured.stl\" />
      </geometry>
    </visual>
    <collision>
      <origin xyz=\"0 0 0\"/>
      <geometry>
        <mesh filename=\"file:///""" + path + """/""" + 'mesh' + """/nontextured.stl\" />
      </geometry>
      <surface>
        <friction>
          <ode>
            <mu>1.0</mu>
            <mu2>1.0</mu2>
          </ode>
        </friction>
        <contact>
          <ode>
            <kp>5000000.0</kp>
            <kd>1.0</kd>
          </ode>
        </contact>
      </surface>
    </collision>
  </link>
  <gazebo reference=\"""" + object_name + """\">
    <material>Gazebo/Red</material>
  </gazebo>
</robot>
"""
f.write(urdf_str)
f.close()
