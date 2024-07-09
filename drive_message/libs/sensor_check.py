import rospy

class FrequencyCheck:
    def __init__(self, topic_name, msg_type, hz_thresh):
        self.last_time = rospy.Time.now()
        self.message_count = 0
        self.hz_thresh = hz_thresh
        self.hz = 0

        rospy.Subscriber(topic_name, msg_type, self.topic_callback)

    def topic_callback(self):
        self.message_count += 1

    def get_hz(self):
        current_time = rospy.Time.now()
        elapsed_time = (current_time - self.last_time).to_sec()
        self.hz = self.message_count / elapsed_time if elapsed_time > 0 else 0
        self.last_time = current_time
        self.message_count = 0
    
    def check(self):
        return self.hz > self.hz_thresh

class ValueCheck:
    def __init__(self, topic_name, msg_type, value_thresh):
    
        rospy.Subscriber(topic_name, msg_type, self.topic_callback)
