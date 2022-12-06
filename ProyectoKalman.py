import sys  
import pygame as pg
import pygame.locals as pgl
import kalman
from utils import ruido

class bola:
    def __init__(self, skin='', escala=(60, 60), pos=(0, 0), vel=(2.5,1.5), tamano_mesa =(1000, 600)):
        self.menu_robot = sys.stdout
        self.teclas = 0
        
        # Tamaño de la mesa de billar
        self.tamano = tamano_mesa
        
        # Visual de la bola de billar
        self.bola = pg.image.load(skin)                       # Carguemos la imagen
        self.bola = pg.transform.scale(self.bola, escala)     # Reescalemos la imagen
        self.bola_encuadre = self.bola.get_rect()             # Borde del rectangulo que describe la imagen
        
        self.angulo_rotacion = 30

        self.bola_encuadre.move_ip((pos[0], pos[1]))    # Se mueve la bola a la posicion establecida
        self.velocidad = [vel[0], vel[1]]               # Arreglo de la velocidad x, y de la bola
        self.posicion = [pos[0], pos[1]]                # Arreglo de la posicion x, y de la bola
    
    def centro(self):
        centrox = self.bola_encuadre.centerx  
        centroy = self.bola_encuadre.centery 
        return centrox, centroy
    
    def automatico(self):
        # Se le asigna al robot una velocidad y se le indica cuáles son los límites del escenario, 
        # restringiendo estas posiciones y haciendo que cambie de rumbo cuando choque con estos puntos.
        self.bola_encuadre = self.bola_encuadre.move(self.velocidad)
        
        # Nueva posicion dada la velocidad
        self.posicion[0] += self.velocidad[0]
        self.posicion[1] += self.velocidad[1]
        # Dado un choque con el borde, cambiar la direccion
        if self.bola_encuadre.left < 0 or self.bola_encuadre.right > self.tamano[0]:
            self.velocidad[0] = -self.velocidad[0]
        if self.bola_encuadre.top < 0 or self.bola_encuadre.bottom > self.tamano[1]:
            self.velocidad[1] = -self.velocidad[1]


class billar():
    def __init__(self, path='./images/', frame=(1000, 600), fondo='', pos=(0,0)):
        self.path = path
        self.bola = self.crear_bola(pos=pos)
        if self.fondo == '':
            self.fondo = fondo
        else:
            self.escenario = pg.image.load(path + fondo)
            self.escenario_rect = self.escenario.get_rect()
        
    def crear_bola(self, img='./bola_ocho.png', pos=(0,0)):
        return bola(img, pos=pos)
        
    def crear_mesa(self):
        self.mesa = pg.display.set_mode((1000,600))
        
    def pintar_mesa_bola(self):
        if type(self.fondo) == tuple:
            self.mesa.fill(self.fondo)
        else:
            self.mesa.blit(self.escenario, self.escenario_rect)
        
        self.mesa.blit(bola.bola, bola.bola_encuadre)
        
    def movimiento_bola(self):
        self.bola.mover()


if __name__ == '__main__':
    import numpy as np
    import matplotlib.pyplot as plt
    
    pg.init()
    bola = bola('./images/bola_ocho.png')
    #print(bola.bola_encuadre)

    # Definimos Kalman y el color de seguimiento
    F = np.array([1])
    H = np.array([1])
    Q = np.array([1e-5])
    R = np.array([0.05])
    
    filtro_kalman_x = kalman.kalman(F, H, Q, R)
    filtro_kalman_y = kalman.kalman(F, H, Q, R)
    filtro_kalman_x.primera_iter()
    filtro_kalman_y.primera_iter()
    
    color_seg = (255, 0, 0)


    fondo = (44, 85, 69)
    screen = pg.display.set_mode((1000,600))
    pg.display.set_caption('Simulacion Kalman')

    #print(bola.centro())
    ground_truth = [[], []]
    estimation = [[], []]

    while True:
        # tiempo muerto milisegundos
        pg.time.delay(2)
        
        # Para poder cerrar la ventana
        for event in pg.event.get():
            if event.type == pgl.QUIT:
                plt.figure(1)

                plt.title('Comparación en X', fontsize=20)
                plt.plot(ground_truth[0], label='Ground truth')
                plt.plot(estimation[0], label='Predicción')
                plt.legend()
                plt.savefig('filter_x.png')
                
                plt.figure(2)

                plt.title('Comparación en Y', fontsize=20)
                plt.plot(ground_truth[1], label='Ground truth')
                plt.plot(estimation[1], label='Predicción')
                plt.legend()
                plt.savefig('filter_y.png')
                
                plt.figure(3)

                plt.title('Comparación en XY', fontsize=20)
                plt.plot(ground_truth[0],ground_truth[1], label='Ground truth')
                plt.plot(estimation[0],estimation[1], label='Predicción')
                plt.legend()
                plt.savefig('filter_xy.png')
                
                plt.show()
                
                x_1 = np.array(ground_truth[0])
                x_2 = np.array(estimation[0])
                y_1 = np.array(ground_truth[1])
                y_2 = np.array(estimation[1])
                
                print('Error promedio en x: ', ((x_1 - x_2)/x_1).sum() / x_1.shape[0])
                print('Error promedio en y: ', ((y_1 - y_2)/y_1).sum() / y_1.shape[0])
                pg.quit()
                sys.exit()
        
        # Dibujar fondo y bola
        screen.fill(fondo)
        screen.blit(bola.bola, bola.bola_encuadre)
        
        # Centro de la bola y agregar ruido
        centro = list(bola.centro())
        medida = ruido(arr=np.array(centro), desv=1)
        
        ground_truth[0].append(centro[0])
        ground_truth[1].append(centro[1])
        
        x_seg, _ = filtro_kalman_x.kalman(medida=medida[0],x0=0,P0=1)
        y_seg, _ = filtro_kalman_y.kalman(medida=medida[1],x0=0,P0=1)
        
        estimation[0].append(x_seg)
        estimation[1].append(y_seg)
        #print(x_seg, y_seg)
        
        pg.draw.rect(screen, color_seg, (x_seg, y_seg, 20, 20))
        
        # Movimeinto de la bola
        bola.automatico()
            
        pg.display.update()
        pg.display.flip()
    pg.quit()