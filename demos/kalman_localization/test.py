from filter import *

class MySystem(LinearSystem):
  def __init__(self):
    '''
    Models the system
    x[k] = A[k] * x[k-1] + B[k] * u[k] + transition noise
    z[k] = C[k] * x[k] + measurement noise
    
    R is the covariance of transition noise
    Q is the covariance of measurement noise

    State vector is
    x = [x y theta]'
    Control vector is
    u = [dist angle]

    x = x_last + dist * cos(theta)
    y = y_last + dist * sin(theta)
    theta = theta_last + angle

    A = C = [[1, 0, 0],
             [0, 1, 0],
             [0, 0, 1]]
    '''
    self._A = np.eye(3)
    self.C = np.eye(3)
    self.R = 0.1 * np.eye(3)
    self.Q = 0.1 * np.eye(3)

  def A(self, x):
      return self._A

  def B(self, x):
      theta = x[2]
      return np.array([[np.cos(theta), 0],
                       [np.sin(theta), 0],
                       [0,             1]])

def main():
    sys = MySystem()
    kf = KalmanFilter(sys, [0, 0, 0], 0.01*np.eye(3))

    print kf.x

    kf.elapseTime([1, np.pi/8])
    kf.elapseTime([1, np.pi/8])
    print kf.x

    kf.observe([1.8, 0.4, np.pi/4-0.1])
    print kf.x

if __name__ == '__main__':
    main()
