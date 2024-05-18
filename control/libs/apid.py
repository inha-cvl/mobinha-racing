import configparser

class APID:
    def __init__(self, ros_handler):
        self.RH = ros_handler
        
        self.set_configs()

        self.error_history = []

        self.dKp, self.dKi, self.dKd = 0, 0, 0
        self.ddKp, self.ddKi, self.ddKd = 0, 0, 0

        self.errs = [-1, -1, -1, -1, -1] # k-3, k-2, k-1, k, k+1 
        self.outs = [-1, -1, -1, -1, -1]
        self.ctrls = [-1, -1, -1, -1, -1]

        self.cnt = 0
        self.ref = 10
        self.cur = 0
        self.epsilon = 1e-6
    
    def set_configs(self):
        config_file_path = './config.ini'
        config = configparser.ConfigParser()
        config.read(config_file_path)
        pid_config = config['PID']
        self.window_size = float(pid_config['window_size'])
        self.Kp = float(pid_config['Kp'])
        self.Ki = float(pid_config['Ki'])/self.window_size
        self.Kd = float(pid_config['Kd'])
        self.lr = float(pid_config['lr'])

    def execute(self):
        if self.RH.system_mode < 1:
            return -3
                
        self.update_history(self.RH.planned_velocity, self.RH.current_velocity)
        self.calculate_ddK()
        self.calculate_dK()
        self.clip_parameters()
        self.update_parameters()

        output = self.calculate_output()

        if output < 0:
            output = (3 / 100) * output #Brake
        else:
            output = (2 / 100) * output #Accel
        return output
    
    def update_history(self, ref, cur):
        for i in range(3):
            self.errs[i] = self.errs[i+1]
            self.outs[i] = self.outs[i+1]
            self.ctrls[i] = self.ctrls[i+1]
        self.errs[3] = ref - cur
        self.outs[3] = cur
        self.ctrls[3] = self.ctrls[4]
        self.error_history.append(ref - cur)
        if len(self.error_history) > self.window_size:
            self.error_history.pop(0)

    def calculate_ddK(self):
        tmp1 = (self.ctrls[1] - self.ctrls[0] + self.epsilon)
        common_term = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / tmp1)
        
        self.ddKp = common_term * (self.errs[2] - self.errs[3] + self.epsilon)
        self.ddKi = common_term * (self.errs[2] + self.epsilon)
        self.ddKd = common_term * (self.errs[2] - 2 * self.errs[1] + self.errs[0] + self.epsilon)

    def calculate_dK(self):
        tmp2 = (self.errs[3] - self.errs[2] + self.epsilon)
        common_term = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / tmp2)

        self.dKp = common_term * (self.errs[3] - self.errs[2] + self.epsilon)
        self.dKi = common_term * (self.errs[3] + self.epsilon)
        self.dKd = common_term * (self.errs[3] - 2 * self.errs[2] + self.errs[1] + self.epsilon)

    def clip_parameters(self):
        plim, ilim, dlim = self.Kp / 10, self.Ki / 10, self.Kd / 10
        self.ddKp = max(-plim, min(self.ddKp, plim))
        self.ddKi = max(-ilim, min(self.ddKi, plim))
        self.ddKd = max(-dlim, min(self.ddKd, dlim))
        self.dKp = max(-plim, min(self.dKp, plim))
        self.dKi = max(-ilim, min(self.dKi, ilim))
        self.dKd = max(-dlim, min(self.dKd, dlim))

    def update_parameters(self):
        self.Kp += self.dKp + self.ddKp
        self.Ki += self.dKi + self.ddKi
        self.Kd += self.dKd + self.ddKd

    def calculate_output(self):
        self.error = self.errs[3]
        self.integral = sum(self.error_history)
        self.derivative = self.errs[3] - self.errs[2]

        output = (self.Kp * self.error) + (self.Ki * self.integral) + (self.Kd * self.derivative)
        self.cur += output
        return max(-100, min(output, 100))

    
