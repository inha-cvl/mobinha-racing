import matplotlib.pyplot as plt
import numpy as np
from time import sleep

class Apid:
    def __init__(self):
        # 일반형
        self.Kp = 0.08
        self.Ki = 0.001
        self.Kd = 0.001
        self.lr = 0.001
        self.error_history = []
        self.window_size = 5

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
    
    def run(self, cur, ref):
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
        tmp2 = (self.errs[3] - self.errs[2])
        if tmp2 == 0:
            tmp2 = 1000 

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
        return output

    def visualize(self):
        plt.ion()
        fig, ax = plt.subplots()
        t, ref_values, cur_values, ctrl_values = [], [], [], []
        cur_t = 0
        while 1:
            control = self.run(self.cur, self.ref)

            # 새로운 값 추가
            t.append(cur_t)
            cur_t += 0.05
            ref_values.append(self.ref)
            cur_values.append(self.cur)
            ctrl_values.append(control)

            # 리스트의 크기가 31이 되면, 가장 오래된 요소(0번 인덱스) 제거
            if len(ref_values) > 30:
                t.pop(0)
                ref_values.pop(0)
                cur_values.pop(0)
                ctrl_values.pop(0)

            ax.clear()
            ax.plot(t, ref_values, label='Reference')
            ax.plot(t, cur_values, label='Current')
            # ax.plot(t, ctrl_values, label='Control', linestyle='--')
            ax.grid(True)
            ax.legend(loc='upper right')
            plt.xlabel('Time step')
            plt.ylabel('Value')
            plt.ylim(0, 12)
            plt.title('Real-time PID Control Visualization')
            plt.pause(0.01)  # 0.05초 동안 일시 정지하여 그래프 업데이트
        plt.ioff()  # 대화형 모드 비활성화
        plt.show()
    
if __name__ == "__main__":

    apid = Apid()
    while 1:
        # apid.run(apid.cur, apid.ref)
        apid.visualize()