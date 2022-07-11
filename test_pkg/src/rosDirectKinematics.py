#!/usr/bin/env python
import rospy
from sensor_msgs.msg import JointState

from markers import *
from mainFunctions import *

np.set_printoptions(precision=4, suppress=True)

rospy.init_node("testForwardKinematics")
pub = rospy.Publisher('joint_states', JointState, queue_size=1)
bmarker = BallMarker(color['RED'])

# Joint names
jnames = ['link1', 'link2', 'link3', 'link4', 'link5', 'link6']
# Joint Configuration
q = [0, -1.57, -0.41, 0, 1.11, 0]
print("Configuracion Deseada:")
print(q)

# End effector with respect to the base
T = fkine(q)
print("\n Matriz de Posicion y Rotacion:")
print(T)
bmarker.position(T)

# Object (message) whose type is JointState
jstate = JointState()
# Set values to the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q

# Loop rate (in Hz)
rate = rospy.Rate(20)
# Continuous execution loop
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    # Wait for the next iteration
    rate.sleep()
