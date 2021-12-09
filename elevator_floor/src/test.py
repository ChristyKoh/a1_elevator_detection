class StateEstimator:
    def __init__():
        # init node
        self.a1_state_sub = rospy.Subscriber(topic_name, msg_type, self,__state_cb)
        self.a1_state_est_timer = rospy.Timer(1/desired_frequency, self.__est_timer_cb)

    def __state_cb(self, msg):
        self.obs_state = msg

    def __est_timer_cb(self, event):
        # do estimation
        dt = 1/self.desired_frequency