# from std_msgs.msg import Float32
# import matplotlib.pyplot as plt
# import rospy

# current_v_history = []
# target_v_history = []
# error_v_history = []
# current_v = 0
# target_v = 0
# def current_v_cb(msg):
#     global current_v
#     current_v = float(msg.data)
#     print(current_v)
#     # current_v_history.append(float(msg.data))

# def target_v_cb(msg):
#     global target_v
#     target_v = float(msg.data)
#     # target_v_history.append(float(msg.data))
#     # print(target_v_history)

# def truncate():
#     global current_v, target_v, error_v_history
#     for arr in (current_v_history, target_v_history, error_v_history):
#         if len(arr) > 20:
#             arr.pop(0)


# if __name__ == "__main__":
#     rospy.init_node("plotter")
#     rospy.Subscriber("/current_v", Float32, current_v_cb)
#     rospy.Subscriber("/target_v", Float32, target_v_cb)

#     plt.ion()  # 대화형 모드 활성화
#     fig, ax = plt.subplots()
#     line1, = ax.plot(current_v_history, 'r-', label='Current Velocity')
#     line2, = ax.plot(target_v_history, 'g-', label='Target Velocity')
#     line3, = ax.plot(error_v_history, 'b-', label='Error')
#     # ax.legend()
#     plt.legend(loc='upper left')
#     plt.grid(True)
#     plt.ylim(0, 25)
#     while not rospy.is_shutdown():
#         current_v_history.append(current_v*3.6)
#         target_v_history.append(target_v*3.6)
#         error_v_history.append((target_v-current_v)*3.6)
#         truncate()

#         line1.set_ydata(current_v_history)
#         line2.set_ydata(target_v_history)
#         line3.set_ydata(error_v_history)
        
#         line1.set_xdata(range(len(current_v_history)))
#         line2.set_xdata(range(len(target_v_history)))
#         line3.set_xdata(range(len(error_v_history)))

#         ax.relim()
#         ax.autoscale_view()

#         plt.draw()
#         plt.pause(0.01)  # 0.1초 간격으로 업데이트
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import rospy
import time

# 초기 시간 기록
start_time = time.time()

current_v_history = []
target_v_history = []
error_v_history = []
current_v = 0
target_v = 0

def current_v_cb(msg):
    global current_v
    current_v = float(msg.data)*3.6
    elapsed_time = time.time() - start_time
    current_v_history.append((elapsed_time, current_v))

def target_v_cb(msg):
    global target_v
    target_v = float(msg.data)*3.6
    elapsed_time = time.time() - start_time
    target_v_history.append((elapsed_time, target_v))

def truncate():
    global current_v, target_v, error_v_history
    for arr in (current_v_history, target_v_history, error_v_history):
        if len(arr) > 20:
            arr.pop(0)

if __name__ == "__main__":
    rospy.init_node("plotter")
    rospy.Subscriber("/current_v", Float32, current_v_cb)
    rospy.Subscriber("/target_v", Float32, target_v_cb)

    plt.ion()  # 대화형 모드 활성화
    fig, ax = plt.subplots()

    lines = {
        'current': ax.plot([], [], 'r-', label='Current Velocity')[0],
        'target': ax.plot([], [], 'g-', label='Target Velocity')[0],
        'error': ax.plot([], [], 'b-', label='Error')[0]
    }

    ax.set_ylim(0, 70) 
    plt.legend(loc='upper left')
    plt.grid(True)

    while not rospy.is_shutdown():
        
        if current_v_history and target_v_history:
            times, current_vals = zip(*current_v_history)
            _, target_vals = zip(*target_v_history)
            error_vals = [t - c for t, c in zip(target_vals, current_vals)]

            if len(times) == len(current_vals) == len(target_vals) == len(error_vals):
                lines['current'].set_data(times, current_vals)
                lines['target'].set_data(times, target_vals)
                lines['error'].set_data(times, error_vals)

                ax.relim()
                ax.autoscale_view()

                plt.draw()
                plt.pause(0.01)  # 0.01초 간격으로 업데이트

            truncate()
