#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import math

class PlazaPath:
    def __init__(self):
        rospy.init_node('plaza_path', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # 10 Hz

    def move_forward(self, distance, speed=0.1):
        move_cmd = Twist()
        move_cmd.linear.x = speed
        duration = distance / speed
        start = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - start) < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(move_cmd)
            self.rate.sleep()
        self.stop_robot()

    def rotate(self, angle_deg, angular_speed=0.3):
        angle_rad = math.radians(angle_deg)
        twist = Twist()
        twist.angular.z = angular_speed if angle_deg > 0 else -angular_speed
        duration = abs(angle_rad / angular_speed)
        start = rospy.Time.now().to_sec()
        while (rospy.Time.now().to_sec() - start) < duration and not rospy.is_shutdown():
            self.cmd_vel_pub.publish(twist)
            self.rate.sleep()
        self.stop_robot()

    def stop_robot(self):
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        rospy.sleep(1)

    def execute(self):
        self.move_forward(0.5)
        self.rotate(90)
        self.move_forward(1.0)
        self.rotate(-90)
        self.move_forward(1.5)
        self.rotate(-90)
        self.move_forward(1.0)
        self.rotate(90)
        self.move_forward(0.5)
        self.rotate(-90)
        self.move_forward(2.0)
        rospy.loginfo("Plaza path completed.")

if __name__ == '__main__':
    try:
        path = PlazaPath()
        rospy.sleep(2)
        path.execute()
    except rospy.ROSInterruptException:
        pass
