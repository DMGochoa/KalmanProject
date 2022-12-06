import numpy as np

def ruido(arr=np.array([]), mu=0, desv=1):
    """En esta funcion se agrega el ruido al arreglo de datos con media mu y desviacion estandar.

    Args:
        arr (np.array, optional): Arreglo de datos que se le va agregar el ruido. Defaults to np.array([]).
        mu (int, optional): Media del ruido. Defaults to 0.
        desv (int, optional): Desviacion estandart del ruido. Defaults to 1.

    Returns:
        np.array: Arreglo de datos con ruido
    """
    return arr + np.random.normal(loc=mu, scale=desv, size=arr.size)

if __name__ == '__main__':
    print(ruido(arr=np.array([30, 30]), desv=1))