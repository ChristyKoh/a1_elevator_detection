from numpy import floor
import rospy

# Define the method which contains the node's main functionality
def floor_pub():

    # new msg type - altitude/floor?
    pub = rospy.Publisher('', Floor, queue_size=10)
    
    # Create a timer object that will sleep long enough to result in a 10Hz
    # publishing rate
    r = rospy.Rate(10) # 10hz

    # Loop until the node is killed with Ctrl-C
    while not rospy.is_shutdown():
        
        pub.publish(floor)
        
        # Use our rate object to sleep until it is time to publish again
        r.sleep()

if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called /talker.
    rospy.init_node('elevator_floor', anonymous=True)

    # Check if the node has received a signal to shut down. If not, run the
    # talker method.
    try:
        floor_pub()
    except rospy.ROSInterruptException: pass