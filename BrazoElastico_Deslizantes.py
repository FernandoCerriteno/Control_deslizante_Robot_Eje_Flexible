#Implementacion de control a un brazo con eje elastico por el metodo de Modos deslizantes
#16/03/2023

#Librerias a utilizar
import numpy as np                  #Funciones matematicas
import math as math                 #Funciones matematicas
from scipy.integrate import odeint  #Solucionador de ecuaciones diferenciales
import matplotlib.pyplot as plt     #Graficador

#Parametros del Robot
m  = 1                      #kg
l  = 1                      #m
Ib = (1/3) * m * pow(l,2)   #ml^2
Bb = 0.001                  #Nms/rad
lc = 0.5                    #m
Jm = 0.008                  #kg m^2
Bm = 0.04                   #N
k  = 50                     #Nm/rad
g  = 9.81                   #m/s^2

#Parametros de Tiempo
inicio  = 0
fin     = 50
paso    = 1e-3
t       = np.arange(inicio, fin, paso)

#Aplicacion del control
def f(e,t):
    #Parametros para la superficie deslizante
    dx_dt   = [0, 0, 0, 0]
    r       = math.sin(t)           #Referencia
    dr      = math.cos(t)           #derivada de r
    d1      = math.exp(-3*t)        #Perturbación aplicada al robot
    d2      = 5 * math.exp(-3*t)    #Perturbacion aplicada al eje del robot
    #Ganancias del control
    k0      = 5.7
    k1      = 4.3
    k2      = 0.15
    k3      = 0.2
    #Superficie deslizante
    s       = k1 * (e[0] - r) + k2 * e[1] + k3 * e[2] + e[3]
    
    #Control equivalente
    Weq     = k1 * (e[1] - dr) + k2 * (- Bb/Ib * e[1] - (m * g * lc)/Ib * math.sin(e[0]) - k/Ib * (e[0] - e[2])  + (1/Ib) * d1) + k3 * e[3]
    
    #Control modos deslizantes
    W       = Weq - k0 * math.fabs(s) * np.sign(s)
    
    #Ley de control
    u           = Jm * ((Bm/Jm) * e[3] + (k/Jm) * (e[2] - e[0]) - d2/Jm + W)
    dx_dt[0]    = e[1]
    dx_dt[1]    = -Bb/Ib * e[1] - (m * g * lc)/Ib * math.sin(e[0]) - k/Ib * (e[0]-e[2]) - d1/Ib
    dx_dt[2]    = e[3]
    dx_dt[3]    = -Bm/Jm * e[3] - k/Jm * (e[2] - e[0]) + (1/Jm) * u  + (1/Jm) * d2
    
    return dx_dt

#Solucion de las ecuaciones diferenciales
sol = odeint(f, y0 = [0,0,0,0], t = t)
print('Solucion ', sol)

#Graficas
#Posición Angular del Robot
plt.plot(t,sol[:,0], 'b', label='x1(t)')
plt.plot(t, np.sin(t), 'r', label='Referencia') #Referencia
plt.xlabel('Tiempo (seg)')
plt.ylabel('x1(t)')
plt.title('Angulo del Robot (rad)')
plt.grid()
plt.show()