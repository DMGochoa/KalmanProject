"""
En este modulo se realiza la implementación de filtros de kalman vista en el nivelatorio.

Diego Alejandro Moreno Gallón
4/10/2022
"""
import matplotlib.animation as animation
import numpy as np
from numpy.linalg import svd

class kalman:
    """
    clase kalman     
    """
    def __init__(self, F, H, Q, R):
        """
        Funcion de inicialización.
        """  
        self.F = F
        self.H = H
        self.Q = Q
        self.R = R
    
        # Se declaran variables para el caso de usar kalman en aplicación real
        self.Xprior = None
        self.Pprior = None
        
    def simulacion(self, medidas, x0, P0):
        """
        Funcion en la que se le ingresa un arreglo que compone la medida y el punto de inicio junto con
        so covariancia.
        """
        X_res = list()
        P_res = list()
        
        for medida in medidas:
            if type(medida) != 'numpy.array':
                ym = np.array([medida])
            else:
                ym = medida

            if medida == medidas[0]:
                Xprior = np.dot(self.F, x0).reshape(-1)
                Pprior = np.dot(self.H, P0) + self.Q
            else:
                Xprior = np.dot(self.F, Xposterior)
                Pprior = np.dot(self.H, Pposterior) + self.Q

            if len(self.H) > 1:
                u, s, v = svd(self.H @ Pprior @ self.H.T + self.R)
                K = Pprior @ self.H.T @ (v.T @ np.identity(len(s))* 1/s @ u.T)
                Xposterior = Xprior + K @ (ym - self.H @ Xprior)
                Pposterior = (np.identity(len(K)) - K @ self.H) @ Pprior
            else:
                K = Pprior * self.H.T * 1/(self.H * Pprior * self.H.T + self.R)
                Xposterior = Xprior + K * (ym - self.H * Xprior)
                Pposterior = (np.identity(len(K)) - K * self.H) * Pprior

            

            X_res.append(Xposterior[0])
            P_res.append(Pposterior[0])
        
        return np.array(X_res), np.array(P_res)
    
    def primera_iter(self):
        self.contador = True
        
    def kalman(self, medida, x0, P0):
        
        if type(medida) != 'numpy.array':
            ym = np.array([medida])
        else:
            ym = medida

        if self.contador:
            #print('entre')
            self.Xprior = np.dot(self.F, x0).reshape(-1)
            self.Pprior = np.dot(self.H, P0) + self.Q
            self.contador = False
        else:
            self.Xprior = np.dot(self.F, self.Xposterior)
            self.Pprior = np.dot(self.H, self.Pposterior) + self.Q

        if len(self.H) > 1:
            u, s, v = svd(self.H @ self.Pprior @ self.H.T + self.R)
            K = self.Pprior @ self.H.T @ (v.T @ np.identity(len(s))* 1/s @ u.T)
            self.Xposterior = self.Xprior + K @ (ym - self.H @ self.Xprior)
            self.Pposterior = (np.identity(len(K)) - K @ self.H) @ self.Pprior
        else:
            K = self.Pprior * self.H.T * 1/(self.H * self.Pprior * self.H.T + self.R)
            self.Xposterior = self.Xprior + K * (ym - self.H * self.Xprior)
            self.Pposterior = (np.identity(len(K)) - K * self.H) * self.Pprior

        return self.Xposterior[0], self.Pposterior[0]