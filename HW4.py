class UserCode:
    
	def __init__(self):
        
		# TODO: tune gains
        
		self.Kp = 10
        
		self.Kd = 5
        
		self.v_measured = 0
            
    
	def compute_control_command(self, t, dt, x_measured, x_desired):
        
		'''
        
		:param t: time since simulation start
        
		:param dt: time since last call to compute_control_command
        
		:param x_measured: measured position (scalar)
        
		:param x_desired: desired position (scalar)
        
		:return - control command u
        
		'''
        
		# TODO: implement PD controller
        
		u = self.Kp*(x_desired-x_measured) + self.Kd*(0-self.v_measured)
        
		self.v_measured += u*dt
                
        
		return u




import numpy as np

class State:
    def __init__(self):
        self.position = np.zeros((3,1))
        self.velocity = np.zeros((3,1))

class UserCode:
    def __init__(self):
        # TODO: tune gains
        self.v_measured = np.zeros((3,1))
        # xy control gains
        Kp_xy = 1 # xy proportional
        Kd_xy = 0.1 # xy differential
        
        # height control gains
        Kp_z  = 1 # z proportional
        Kd_z  = 0.1 # z differential
        
        self.Kp = np.array([[Kp_xy, Kp_xy, Kp_z]]).T
        self.Kd = np.array([[Kd_xy, Kd_xy, Kd_z]]).T
    
    def compute_control_command(self, t, dt, state, state_desired):
        '''
        :param t: time since simulation start
        :param dt: time since last call to measurement_callback
        :param state: State - current quadrotor position and velocity computed from noisy measurements
        :param state_desired: State - desired quadrotor position and velocity
        :return - xyz velocity control signal represented as 3x1 numpy array
        '''
        # plot current state and desired setpoint
        self.plot(state.position, state_desired.position)
        
        # TODO: implement PID controller computing u from state and state_desired
        err =self.Kp*(state_desired.position-state.position)
        u = err + self.Kd*(state_desired.velocity-state.velocity)
        
        return u
        
    def plot(self, position, position_desired):
        from plot import plot
        plot("x", position[0])
        plot("x_des", position_desired[0])
        plot("y", position[1])
        plot("y_des", position_desired[1])
        plot("z", position[2])
        plot("z_des", position_desired[2])
        
