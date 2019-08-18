from numpy.linalg import inv
import numpy as np

matrix = [
    [ 0.01334171, -0.99899336, -0.04282841],
    [-0.53038226,  0.02923977, -0.84725421],
    [ 0.84765362,  0.03401925, -0.52945825]
]

print(inv(np.matrix(matrix)))
