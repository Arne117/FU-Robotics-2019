from numpy.linalg import inv
import numpy as np

matrix = [
 [-0.65935479, -0.03150184,  0.75117168],
 [ 0.16506872, -0.98080894,  0.10375999],
 [ 0.73348727,  0.1924096 ,  0.65190104]
]

print(inv(np.matrix(matrix)))
