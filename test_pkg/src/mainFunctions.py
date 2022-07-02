import numpy as np
from copy import copy
from tf.transformations import quaternion_from_matrix
from pyquaternion import Quaternion

cos = np.cos
sin = np.sin
pi = np.pi

def dh(d, theta, a, alpha):
    """
    Calcular la matriz de transformacion homogenea asociada con los parametros
    de Denavit-Hartenberg.
    Los valores d, theta, a, alpha son escalares.
    """
    # Escriba aqui la matriz de transformacion homogenea en funcion de los valores de d, theta, a, alpha
    T = np.array([[cos(theta), -cos(alpha)*sin(theta),  sin(alpha)*sin(theta), a*cos(theta)],
                  [sin(theta),  cos(alpha)*cos(theta), -sin(alpha)*cos(theta), a*sin(theta)],
                  [0         ,  sin(alpha)           ,  cos(alpha)           , d],
                  [0         ,  0                    ,  0                    , 1]])
    return T


def fkine(q):
    """
    Calcular la cinematica directa del robot UR5 dados sus valores articulares. 
    q es un vector numpy de la forma [q1, q2, q3, q4, q5, q6]
    """
    # Longitudes (en metros)

    # Matrices DH (completar), emplear la funcion dh con los parametros DH para cada articulacion
    T0 = dh(0.076   ,         0,      0,     0)
    T1 = dh(0.035   , q[0]+pi/2,  0.035,  pi/2)
    T2 = dh(0.000   , q[1]+pi/2,  0.146,    pi)
    T3 = dh(0.000   ,      q[2],  0.052, -pi/2)
    T4 = dh(0.115   ,      q[3],      0, -pi/2)
    T5 = dh(0.000   ,      q[4],      0,  pi/2)
    T6 = dh(0.100   ,      q[5],      0,     0) # 0.082

    # Efector final con respecto a la base
    T = T0.dot(T1).dot(T2).dot(T3).dot(T4).dot(T5).dot(T6)
    return T
    
def jacobian(q, delta=0.0001):
    """
    Jacobiano analitico para la posicion. Retorna una matriz de 3x6 y toma como
    entrada el vector de configuracion articular q=[q1, q2, q3, q4, q5, q6]
    """
    # Crear una matriz 3x6
    J = np.zeros((3, 6))
    # Transformacion homogenea inicial (usando q)
    Ti = fkine(q)

    # Iteracion para la derivada de cada columna
    for i in range(6):
        # Copiar la configuracion articular inicial
        dq = copy(q)
        # Incrementar la articulacion i-esima usando un delta
        dq[i] = dq[i] + delta
        # Transformacion homogenea luego del incremento (q+delta)
        Tf = fkine(dq)
        # Aproximacion del Jacobiano de posicion usando diferencias finitas
        J[0,i] = (Tf[0,3]-Ti[0,3])/delta
        J[1,i] = (Tf[1,3]-Ti[1,3])/delta
        J[2,i] = (Tf[2,3]-Ti[2,3])/delta
    return J


def ikine(xdes, q0):
    """
    Calcular la cinematica inversa de UR5 numericamente a partir de la configuracion articular inicial de q0. 
    Emplear el metodo de newton
    """
    epsilon = 0.001
    max_iter = 1000
    delta = 0.00001

    q = copy(q0)
    for i in range(max_iter):
        J = jacobian(q, delta)
        T = fkine(q)
        f = T[0:3,3]
        e = xdes - f
        q = q + np.dot(J.T, e)
        if(np.linalg.norm(e) < epsilon):
            break
    return q
