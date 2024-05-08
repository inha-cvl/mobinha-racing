from std_msgs.msg import Float32
import matplotlib.pyplot as plt
import rospy
import time

# 초기 시간 기록
start_time = time.time()

current_v_history = []
current_v2_history = []  # Additional history for current_v2
target_v_history = []
error_v_history = []
current_v = 0
current_v2 = 0  # New current velocity 2 variable
target_v = 0

def current_v_cb(msg):
    global current_v
    current_v = float(msg.data) * 3.6
    elapsed_time = time.time() - start_time
    current_v_history.append((elapsed_time, current_v))

def current_v2_cb(msg):  # New callback for current_v2
    global current_v2
    current_v2 = float(msg.data)
    elapsed_time = time.time() - start_time
    current_v2_history.append((elapsed_time, current_v2))
    print(current_v2)

def target_v_cb(msg):
    global target_v
    target_v = float(msg.data) * 3.6
    elapsed_time = time.time() - start_time
    target_v_history.append((elapsed_time, target_v))

def truncate():
    global current_v, current_v2, target_v, error_v_history
    for arr in (current_v_history, current_v2_history, target_v_history, error_v_history):
        if len(arr) > 20:
            arr.pop(0)

if __name__ == "__main__":
    rospy.init_node("plotter")
    rospy.Subscriber("/current_v", Float32, current_v_cb)
    rospy.Subscriber("/current_v2", Float32, current_v2_cb)  # Subscriber for current_v2
    rospy.Subscriber("/target_v", Float32, target_v_cb)

    plt.ion()  # 대화형 모드 활성화
    fig, ax = plt.subplots()

    lines = {
        'current': ax.plot([], [], 'r-', label='Current Velocity')[0],
        'current2': ax.plot([], [], 'm-', label='Current Velocity 2')[0],  # Line for current_v2
        'target': ax.plot([], [], 'g-', label='Target Velocity')[0],
        'error': ax.plot([], [], 'b-', label='Error')[0]
    }

    ax.set_ylim(0, 30)
    plt.legend(loc='upper left')
    plt.grid(True)

    while not rospy.is_shutdown():
    
        if current_v_history and current_v2_history and target_v_history:
            times, current_vals = zip(*current_v_history)
            _, current2_vals = zip(*current_v2_history)
            _, target_vals = zip(*target_v_history)
            error_vals = [t - c for t, c in zip(target_vals, current_vals)]

            if len(times) == len(current_vals) == len(current2_vals) == len(target_vals) == len(error_vals):
                lines['current'].set_data(times, current_vals)
                lines['current2'].set_data(times, current2_vals)  # 여전히 같은 times 배열 사용
                lines['target'].set_data(times, target_vals)
                lines['error'].set_data(times, error_vals)

                ax.relim()
                ax.autoscale_view()

                plt.draw()
                plt.pause(0.01)  # 0.01초 간격으로 업데이트