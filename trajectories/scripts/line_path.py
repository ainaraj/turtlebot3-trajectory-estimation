#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

class MoveForward:
    def __init__(self):
        rospy.init_node('move_forward', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

    def move_forward(self, distance, speed=0.2):
        """ Moves the robot forward for a given distance """
        move_cmd = Twist()
        move_cmd.linear.x = speed

        move_time = distance / speed  
        start_time = rospy.Time.now().to_sec()

        rospy.loginfo(f"Moving forward for {move_time:.2f} seconds...")

        while (rospy.Time.now().to_sec() - start_time) < move_time and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(move_cmd) 
            self.rate.sleep()  

        self.stop_robot()

    def stop_robot(self):
        """ Stops the robot """
        rospy.loginfo("Stopping robot...")
        stop_cmd = Twist()  
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.sleep(1)  

    def execute(self):
        self.move_forward(8)  # Move forward 8 meters
        rospy.loginfo("Movement complete.")

if __name__ == '__main__':
    try:
        bot = MoveForward()
        rospy.sleep(2)  
        bot.execute()
    except rospy.ROSInterruptException:
        rospy.loginfo("Move Forward node terminated.")
