import rbdl
import numpy as np

np.set_printoptions(precision=4, suppress=True)

# Lectura del modelo del robot a partir de URDF (parsing)
modelo = rbdl.loadModel(
    #'/mnt/c/ubuntu18/lab_ws/src/frlabs/lab7/urdf/ur5_robot.urdf')
    '/mnt/c/ubuntu18/ProyectoFundamentos/proyectoWS/src/test_pkg/urdf/R6DOF.urdf')
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
e     = np.eye(6)               # Vector identidad

# Torque dada la configuracion del robot
rbdl.InverseDynamics(modelo, q, dq, ddq, tau)
print("Vector torque: \n{}".format(tau))

# Parte 1: Calcular vector de gravedad, vector de Coriolis/centrifuga,
# y matriz M usando solamente InverseDynamics

# Vector gravedad
rbdl.InverseDynamics(modelo, q, zeros, zeros, g)
print("Vector gravedad: \n{}".format(g))

# Vector coriolis
rbdl.InverseDynamics(modelo, q, dq ,zeros, c)
c = c - g
print("Vector coriolis: \n{}".format(c))

# Matriz inercia
for i in range(ndof):
    rbdl.InverseDynamics(modelo, q, zeros, e[i,:], M[i,:])
    M[i,:] = M[i,:] - g
print("Matriz inercia: \n{}".format(M))

# Parte 2: Calcular M y los efectos no lineales b usando las funciones
# CompositeRigidBodyAlgorithm y NonlinearEffects. Almacenar los resultados
# en los arreglos llamados M2 y b2

b2 = np.zeros(ndof)          # Para efectos no lineales
M2 = np.zeros([ndof, ndof])  # Para matriz de inercia





# Parte 2: Verificacion de valores





# Parte 3: Verificacion de la expresion de la dinamica