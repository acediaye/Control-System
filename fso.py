import control
import numpy as np
import matplotlib.pyplot as plt

class FSO(object):
    def __init__(self, eigenvalues: np.ndarray):
        self.poles_desire = eigenvalues
        self.ss_plant = None  # state space plant
        
        # save response values
        self.time_out = None
        self.y_ol_out = None
        self.y_cl_out = None
        self.y_kr_out = None
        self.x_kr_out = None
        self.u_kr_out = None
        self.reference = None
    
    def excite(self, plant: control.StateSpace, time: np.ndarray, reference: np.ndarray) -> tuple:
        self.ss_plant = plant
        A, B, C, D = control.ssdata(plant)
        if len(self.poles_desire) != np.shape(A)[0]:  # check n
            raise RuntimeError('missing eigenvalues')
        obsv = control.obsv(A, C)
        rank_o = np.linalg.matrix_rank(obsv)
        if rank_o != np.shape(A)[0]:
            raise RuntimeError('not full row rank')
        
        # open loop response
        self.time_out, self.y_ol_out = control.forced_response(self.ss_plant, time, reference)
        K = control.place(A, B, self.poles_desire)
        eigval, eigvec = np.linalg.eig(A-B@K)
        print(eigval)
        # Ao = A-L@C
    
    def graph(self, save):
        pass
    
    def pzmap(self):
        pass