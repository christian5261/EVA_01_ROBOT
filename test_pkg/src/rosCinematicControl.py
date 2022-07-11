#!/usr/bin/env python
#

from __future__ import print_function
import rospy
from sensor_msgs.msg import JointState

from markers import *
from mainFunctions import *

np.set_printoptions(precision=4, suppress=True)

# Initialize the node
rospy.init_node("testKineControlPosition")
print('starting motion ... ')
# Publisher: publish to the joint_states topic
pub = rospy.Publisher('joint_states', JointState, queue_size=10)
# Files for the logs
fxcurrent = open(
    "/mnt/c/ubuntu18/ProyectoFundamentos/imagenesProyecto/xcurrent.txt", "w")
fxdesired = open(
    "/mnt/c/ubuntu18/ProyectoFundamentos/imagenesProyecto/xdesired.txt", "w")
fq = open(
    "/mnt/c/ubuntu18/ProyectoFundamentos/imagenesProyecto/q.txt", "w")
fe = open(
    "/mnt/c/ubuntu18/ProyectoFundamentos/imagenesProyecto/e.txt", "w")

# Markers for the current and desired positions
bmarker_current = BallMarker(color['RED'])
bmarker_desired = BallMarker(color['GREEN'])

# Joint names
jnames = ['link1', 'link2', 'link3', 'link4', 'link5', 'link6']

# Desired position
xd = np.array([0.214, 0.125, 0.1])
print("Posicion deseada: ")
print(xd)
# Initial configuration
q0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
print("\nConfiguracion inicial: ")
print(q0)

# Resulting initial position (end effector with respect to the base link)
T = fkine(q0)
x0 = T[0:3, 3]
print("Matriz de posicion y orientacion inicial: ")
print(T)

# Red marker shows the achieved position
bmarker_current.xyz(x0)
# Green marker shows the desired position
bmarker_desired.xyz(xd)

# Instance of the JointState message
jstate = JointState()
# Values of the message
jstate.header.stamp = rospy.Time.now()
jstate.name = jnames
# Add the head joint value (with value 0) to the joints
jstate.position = q0

# Frequency (in Hz) and control period
freq = 200
dt = 1.0/freq
rate = rospy.Rate(freq)

# Initial joint configuration
q = copy(q0)
# Main loop
k = 0.5
i = 0
while not rospy.is_shutdown():
    # Current time (needed for ROS)
    jstate.header.stamp = rospy.Time.now()
    # Kinematic control law for position (complete here)
    # -----------------------------
    T = fkine(q)
    x = T[0:3,3]
    e = x - xd
    ep = -k*e
    enorm = np.linalg.norm(e)
    J = jacobian(q)
    qp = np.linalg.pinv(J).dot(ep)
    q = q + dt*qp
    # -----------------------------
    if(enorm < 0.001):
        print("Posicion deseada en: {} s".format(i*dt))
        break
    i += 1
    
    # Log values
    fxcurrent.write(str(x[0])+' '+str(x[1]) + ' '+str(x[2])+'\n')
    fxdesired.write(str(xd[0])+' '+str(xd[1])+' '+str(xd[2])+'\n')
    fq.write(str(q[0])+" "+str(q[1])+" "+str(q[2])+" "+str(q[3])+" " +
             str(q[4])+" "+str(q[5])+"\n")
    fe.write(str(e[0])+" "+str(e[1])+" "+str(e[2])+" "+str(enorm)+"\n")

    # Publish the message
    jstate.position = q
    pub.publish(jstate)
    bmarker_desired.xyz(xd)
    bmarker_current.xyz(x)
    # Wait for the next iteration
    rate.sleep()

print('ending motion ...')
fxcurrent.close()
fxdesired.close()
fq.close()
fe.close()
