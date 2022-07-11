#!/usr/bin/env python

import rbdl
import numpy as np

np.set_printoptions(precision=4, suppress=True)

# Lectura del modelo del robot a partir de URDF (parsing)
modelo = rbdl.loadModel(
    '/mnt/c/ubuntu18/ProyectoFundamentos/proyectoWS/src/test_pkg/urdf/R6DOF2.urdf')
# Grados de libertad
ndof = modelo.q_size

# Configuracion articular
q = np.array([0.5, 0.2, 0.3, 0.8, 0.5, 0.6])
# Velocidad articular
dq = np.array([0.8, 0.7, 0.8, 0.6, 0.9, 1.0])
# Aceleracion articular
ddq = np.array([0.2, 0.5, 0.4, 0.3, 1.0, 0.5])

# Arrays numpy
zeros = np.zeros(ndof)          # Vector de ceros
tau   = np.zeros(ndof)          # Para torque
g     = np.zeros(ndof)          # Para la gravedad
c     = np.zeros(ndof)          # Para el vector de Coriolis+centrifuga
M     = np.zeros([ndof, ndof])  # Para la matriz de inercia
e     = np.eye(ndof)               # Vector identidad

# Torque dada la configuracion del robot
rbdl.InverseDynamics(modelo, q, dq, ddq, tau)
print("Vector torque: \n{}".format(10e3*tau))

# Vector gravedad
rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
print("Vector gravedad: \n{}".format(10e3*g))

# Vector coriolis
rbdl.InverseDynamics(modelo, q, dq ,zeros, c)
c = c - g
print("Vector coriolis: \n{}".format(10e3*c))

# Matriz inercia
for i in range(ndof):
    rbdl.InverseDynamics(modelo, q, zeros, e[i,:], M[i,:])
    M[i,:] = M[i,:] - g
print("Matriz inercia: \n{}".format(10e5*M))