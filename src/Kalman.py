import numpy as np

class Kalman(object):
    DIMENSIONS = 3
    MEASUREMENT = 1
   
    def __init__(self, q, r):
        #initialise
        self.Q = np.matrix(np.eye(Kalman.DIMENSIONS)*q)
        self.R = np.matrix(np.eye(Kalman.MEASUREMENT)*r)
        self.H = np.matrix(np.zeros((Kalman.MEASUREMENT, Kalman.DIMENSIONS)))
       
        for i in range(Kalman.MEASUREMENT):
            self.H[i, i] = 1.0
       
       
        #state
        self.x = np.matrix(np.zeros((Kalman.DIMENSIONS, 1)))
        self.P = np.matrix(np.eye(Kalman.DIMENSIONS))
   
   
    def make_A(self, dt):
        A = np.matrix(np.eye(Kalman.DIMENSIONS))
       
        for i in range(Kalman.MEASUREMENT):
            A[i, Kalman.MEASUREMENT+i] = dt
       
        return A
   
   
    def predict(self, dt):
        A = self.make_A(dt)
       
        x = A*self.x
        P = A*self.P*A.T + self.Q
       
        return x, P
   
   
    def update(self, z, dt):
        x_p, P_p = self.predict(dt)
       
       
        K = P_p*self.H.T*(self.H*P_p*self.H.T + self.R).I
        self.x = x_p + K*(z - self.H*x_p)
        self.P = (np.matrix(np.eye(Kalman.DIMENSIONS)) - K*self.H)*P_p
       
        return self.position(), self.velocity()
   
   
    def update_without_measurement(self, dt):
        self.x, self.P = self.predict(dt)
       
        return self.position(), self.velocity()
   
   
    def position(self):
        return self.x[0:Kalman.MEASUREMENT]
   
   
    def velocity(self):
        return self.x[Kalman.MEASUREMENT:2*Kalman.MEASUREMENT]

if __name__ == "__main__":
    """
    Kalman Filter Test
    """
    k = Kalman(1, 0.2)
    
    for x in range(10):
        k.update([1.0,2.0,3.0], 0.1)
        print x, " Pos: ", k.position()
        print x, " Velo: ", k.velocity()
        print 25*"="
