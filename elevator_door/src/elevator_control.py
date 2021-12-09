import rospy
from elevator_door.msg import ElevatorDoorState
from unitree_legged_msgs.msg import HighCmd

class ElevatorControl:
    
    def __init__(self, floor):
        self.door_sub = rospy.Subscriber('/elevator/door_state', ElevatorDoorState)
        # self.floor_sub = rospy.Subscriber('/elevator/floor', ElevatorDoorState)
        self.state_pub = rospy.Publisher('/')

        self.current_floor = floor
        self.target_floor = floor

    def set_target(self, floor):
        self.target_floor = floor

    def set_current(self,floor):
        self.current_floor = floor

    # ask: will i need to code in CPP?
    

if __name__ == '__main__':
    rospy.init_node('elevator_control')
