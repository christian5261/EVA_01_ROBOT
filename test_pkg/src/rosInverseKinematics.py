#!/usr/bin/env python

import rospy
from sensor_msgs.msg import JointState
from markers import *
from mainFunctions import *

np.set_printoptions(precision=4, suppress=True)

rospy.init_node("testInvKine")
pub = rospy.Publisher('joint_states', JointState, queue_size=1000)

bmarker = BallMarker(color['RED'])
bmarker_des = BallMarker(color['GREEN'])

# Joint names
jnames = ['link1', 'link2', 'link3', 'link4', 'link5', 'link6']

# Desired position
xd = np.array([0.2, -0.4, 0.8])
print("Posicion deseada: ")
print(xd)
# Initial configuration
q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
print("Configuracion inicial: ")
print(q0)
# Inverse kinematics
q = ikine(xd, q0)
print("Configuracion final obtenida:")
print(q)

# Resulting position (end effector with respect to the base link)
T = fkine(q)
print("\nMatriz de posicion y orientacion: ")
print(T)

# Red marker shows the achieved position
bmarker.xyz(T[0:3, 3])
# Green marker shows the desired position
bmarker_des.xyz(xd)

# Objeto (mensaje) de tipo JointState
jstate = JointState()
# Asignar valores al mensaje
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q

# Loop rate (in Hz)
rate = rospy.Rate(100)
# Continuous execution loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    bmarker_des.publish()
    # Wait for the next iteration
    rate.sleep()
