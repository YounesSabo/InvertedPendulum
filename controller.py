from utilities import *

class Controller:
    "Your Controller"
    def __init__(self):
        self.k = 8
        
        # the zeros used in the controller!
        self.a = 0
        self.b = 0
        
        # error and previous u's used in the lineair equation
        self.u_prev = 0
        self.e_prev = 0
        self.e_prev2 = 0
        
        # angle that we want our pendulum on
        self.angle = 0
        
        self.N_SAMPELS = 0

        # the observer
        self.x_hat = np.array([[0.0], [0.0],[0.0], [0.0]])
        self.observer = Observer()
        
        # State feedback
        self.K = 0
        self.u = 0
        
        # Extrended state feedback
        self.Ki = 0
        self.Ke = 0
        self.pos = 0
        self.SE_prev = 0
        self.PI = 0 # When 0 ==> working with I-controller
        self.Kp = 0 
        self.Kcorr = 0
    def set_params(self, parameters):
        "params from matlab set_mode_params"
        self.angle = parameters[0]
        if params.mode == 'CLASSICAL':
            
            self.k = parameters[1]
            self.u_prev = params.w
            # Will assign the values used for the zeros of the controller in matlab
            self.a = parameters[2]
            self.b = parameters[3]
            self.N_SAMPELS = parameters[4]
            
        elif params.mode == 'STATE_SPACE':
            
            self.Kcl = parameters[1]
            self.N_SAMPELS = parameters[2]
            self.K = np.array(parameters[3:])
            print(self.K)
            
        elif params.mode == 'EXTENDED':
            
            self.pos = parameters[1]
            self.PI = parameters[2]
            
            if self.PI == 1:
                print('Working with a PI controller and extended feedback')
                self.Ki = parameters[3]
                self.N_SAMPELS = parameters[4]
                self.Kp = parameters[5]
                self.Kcorr = np.array(parameters[6:])
                print(self.Kcorr)
            else: 
                print('Working with an I controller and extended feedback')
                self.Ki = parameters[3]
                self.N_SAMPELS = parameters[4]
                self.Ke = np.array(parameters[5:])
                print(self.Ke)
        self.observer = Observer()
        
        
        
    def __call__(self, y, n_samples):
        """Call controller with measurement y
        This method is called by the system.System class
        Returns:
            u (float): the command signal for the system
            l (list): a list of values that is returned to matlab
        """
        
        
        if params.mode == 'OPEN_LOOP':
            u = params.w
        elif params.mode == 'CLASSICAL':    
                                
            # y is a list thus we have to make sure that we use one of the param of y to make the error
            # We prefere to control the angle of the pendulum thus we work with y[1]
            
            # We will work in 2 situation, cause when startin the system.py file and the controller.py file our simulation will never rerun __init__ . Thus when 
            # we want to make another measurement we have to make sure that we reset the whole system. The controller.py will know that a new measurement is started when 
            # self.N_SAMPELS == the original amount of samples.
            
            # utilities.py will keep track of the current index of the sampels, thus when we make the measurement of the first samples we want to make sure that the system is reseted. 
            # That's why, when they are both equal, we wouldn't let the controller have any effect on the first measuerement
            if self.N_SAMPELS == n_samples:
                self.u = params.w
                self.u_prev = self.u
                self.e_prev2 = 0
                self.e_prev = 0
            else: 
                error = self.angle - y[1]
                A = self.a
                B = self.b
                self.u = self.u_prev + self.k*(error - (A + B)*self.e_prev + A*B*self.e_prev2)
                # anti windup
                u_treshhold = 10

                if abs(u) > u_treshhold:
                    self.u = np.sign(self.u)*u_treshhold

                    error = (u-self.u_prev)/self.k + (A + B)*self.e_prev - A*B*self.e_prev2
                # reassigning the next values 
                self.u_prev = self.u
                self.e_prev2 = self.e_prev
                self.e_prev = error


            
                
            
        elif params.mode == 'STATE_SPACE':
            # Hier moeten we de observer roepen met de u_prev en gaan we u opstellen met de observatie
            if self.N_SAMPELS == n_samples:
                self.u = params.w
                print('Restarting the measurements with F = ',self.u,'N')
            else: 
                self.x_hat = self.observer(self.u,y)
                self.u = -self.K.dot(self.x_hat)                
                self.u = self.u[0]
                    
        elif params.mode == 'EXTENDED':
            
            if self.N_SAMPELS == n_samples:
                self.u = params.w
                print('Restarting the measurements with a force = ',self.u,'N')
                self.SE_prev = 0
                # print('SE_prev = ',self.SE_prev) 
            else: 
                if self.PI == 1:
                
                    self.x_hat = self.observer(self.u,y)
                    
                    SE = self.SE_prev + (y[0] - self.pos) # is actually -SE

                    #u_i = -self.Ki * SE - self.Kp*(self.pos - y[0]) 
                    
                    #u_hat = -self.Ke.dot(self.x_hat)
                    
                    #self.u = np.add(u_i , u_hat)
                    
                    self.u = -self.Kcorr.dot(self.x_hat) - self.Ki*SE + self.Kp*self.pos
                                
                    self.u = self.u[0]
                    
                    self.SE_prev = SE
                else:
                    
                    self.x_hat = self.observer(self.u,y)
                    
                    SE = self.SE_prev + (y[0] - self.pos) 

                    u_i = -self.Ki * SE  
                    
                    u_hat = -self.Ke.dot(self.x_hat)
                    
                    self.u = np.add(u_i , u_hat)
                                
                    self.u = self.u[0]
                    
                    self.SE_prev = SE
        else:
            self.u = 0.0
        #print(f"y = {y:5.3f}, e = {error:5.3f}, u = {u:5.3f}")

        
        
        # Origineel stond er [u,y] ma we hebben 2 verschillende metingen die we aanbrengen
        # We moeten dus de metingen appart doorsturen en niet de array van de metingen aangezien dat voor een error zal zorgen in utilities.py
        return self.u,[self.u,y[0],y[1],self.x_hat[0], self.x_hat[1],self.x_hat[2], self.x_hat[3]]

class Observer:
    "Implement your observer"
    def __init__(self):
            # theta system
            self.A_obs_theta = np.array([[-0.6911 ,0.0498 ,0.006304 ,2.852e-05],[-14.17 ,0.9919   ,0.0957 ,0.001712],[0.02169 ,-0.0002909, -0.6208, 0.05033],[0.4972, -0.01166, -13.18, 1.02]])
            self.B_obs_theta = np.array([[0.00203, 1.691, -0.004592], [0.08113 ,14.17 ,-0.02709],[0.002909, -0.02169,  1.641],[0.1166, -0.4972, 13.98]])
            self.x_hat_theta = np.array([[0],[0],[0],[0]])

            # x system
            self.A_obs_x = np.array([[0.01981 ,0.0498 ,-0.1053 ,2.852e-05],[-3.68 ,0.9919 ,-1.033 ,0.001712],[-0.2743,-0.0002909 ,-0.03172 ,0.05033],[-2.725,-0.01166,-5.14,1.02]])
            self.B_obs_x = np.array([[0.00203 ,0.9802 ,0.107], [0.08113, 3.68, 1.101],[0.002909, 0.2743, 1.052],[0.1166, 2.725, 5.943]])
            self.x_hat_x = np.array([[0],[0],[0],[0]])
    def __call__(self, u, y):
        "Call observer with this method; Inputs: command u and measurement y"
        u_hat = np.array([[u],[y[0]],[y[1]]])

        self.x_hat_theta = self.A_obs_theta.dot(self.x_hat_theta) + self.B_obs_theta.dot(u_hat)

        self.x_hat_x = self.A_obs_x.dot(self.x_hat_x) + self.B_obs_x.dot(u_hat)
        
        x_hat = np.array([[0.0],[0.0],[0.0],[0.0]])

        x_hat[0] = self.x_hat_x[0]
        x_hat[1] = self.x_hat_x[1]
        x_hat[2] = self.x_hat_theta[2]
        x_hat[3] = self.x_hat_theta[3]

        return x_hat
#### ------ don't change anything below ----------- ####

if __name__ == '__main__': 
    params = Params()
    if len(sys.argv) > 1:
        params.ip = int(sys.argv[1])
    if len(sys.argv) > 2:
        params.Ts = float(sys.argv[2])
    
    print(f'Sampling period = {params.Ts}\nListening on port {params.ip}')
    controller = Controller()

    sys_comms_thread = threading.Thread(target=system_comms, args=(controller, params))
    sys_comms_thread.start()

    matlab_comms_thread = threading.Thread(target=matlab_comms, args=(controller, params))
    matlab_comms_thread.start()