class APID:
    def __init__(self, ros_handler):
        self.RH = ros_handler

        # 일반형
        self.window_size = 10
        self.Kp = 20
        self.Ki = 5/self.window_size
        self.Kd = 9
        self.lr = 0.001
        self.error_history = []

        # 증분형
        # self.Kp = 0.75
        # self.Ki = 0.03
        # self.Kd = 0.05
        # self.lr = 0.01

        self.dKp = 0
        self.dKi = 0
        self.dKd = 0

        self.ddKp = 0
        self.ddKi = 0
        self.ddKd = 0

        self.errs = [-1, -1, -1, -1, -1] # k-3, k-2, k-1, k, k+1 
        self.outs = [-1, -1, -1, -1, -1]
        self.ctrls = [-1, -1, -1, -1, -1]

        self.cnt = 0

        self.ref = 10
        self.cur = 0

        self.epsilon = 1e-6
    

    def execute(self):
        if self.RH.system_status < 1:
            return -3

        ref = self.RH.target_velocity
        cur = self.RH.current_velocity

        # update
        for i in range(3):
            self.errs[i] = self.errs[i+1]
            self.outs[i] = self.outs[i+1]
            self.ctrls[i] = self.ctrls[i+1]
        self.errs[3] = ref - cur
        self.outs[3] = cur
        self.ctrls[3] = self.ctrls[4]
        self.error_history.append(ref - cur)
        if len(self.error_history)>self.window_size:
            self.error_history.pop(0)

        # ddK(k-1)
        tmp1 = (self.ctrls[1] - self.ctrls[0] + self.epsilon)
        
        self.ddKp = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / tmp1) * \
                    (self.errs[2] - self.errs[3] + self.epsilon)
        self.ddKi = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / tmp1) * \
                    (self.errs[2] + self.epsilon)
        self.ddKd = (-self.lr) * (-self.errs[2]) * ((self.outs[1] - self.outs[0]) / tmp1) * \
                    (self.errs[2] - 2*self.errs[1] + self.errs[0] + self.epsilon)
        # dK(k)
        tmp2 = (self.errs[3] - self.errs[2] + self.epsilon)

        self.dKp = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / tmp2) * \
                    (self.errs[3] - self.errs[2] + self.epsilon)
        self.dKi = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / tmp2) * \
                    (self.errs[3] + self.epsilon)
        self.dKd = (-self.lr) * (-self.errs[3]) * ((self.outs[2] - self.outs[1]) / tmp2) * \
                    (self.errs[3] - 2*self.errs[2] + self.errs[1] + self.epsilon)
        
        # clip 
        plim = self.Kp/10
        ilim = self.Ki/10
        dlim = self.Kd/10
        self.ddKp = max(-plim, min(self.ddKp, plim))
        self.ddKi = max(-ilim, min(self.ddKi, plim))
        self.ddKd = max(-dlim, min(self.ddKd, plim))
        self.dKp = max(-plim, min(self.dKp, plim))
        self.dKi = max(-ilim, min(self.dKi, plim))
        self.dKd = max(-dlim, min(self.dKd, plim))
        
        # K(k+1)
        Kp = self.Kp + self.dKp + self.ddKp
        Ki = self.Ki + self.dKi + self.ddKi
        Kd = self.Kd + self.dKd + self.ddKd

        # control_amount : 일반형
        self.error = self.errs[3]
        self.integral = sum(self.error_history)
        self.derivative = self.errs[3]-self.errs[2]

        output = (Kp * self.error) + (Ki * self.integral) + (Kd *self.derivative)
        self.cur += output

        # control_amount : 증분형
        # delta_up = (Kp * (self.errs[3] - self.errs[2]))
        # delta_ui = (Ki * self.errs[3])
        # delta_ud = (Kd * (self.errs[3] - 2*self.errs[2] + self.errs[1]))
        # delta_u_k = delta_up + delta_ui + delta_ud

        # self.ctrls[4] = self.ctrls[3] + delta_u_k
        # output = self.ctrls[4]
        # self.cur += self.ctrls[4] # for plot

        output = max(-100, min(output, 100))
        # output mapping : output -> brake, accel
        if output < 0:
            output = (3 / 100) * output
        else:
            output = (2 / 100) * output
        return output