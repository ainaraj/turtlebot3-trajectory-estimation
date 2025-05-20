#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist

def stop_robot(pub):
    stop_cmd = Twist()
    stop_cmd.linear.x = 0.0
    stop_cmd.angular.z = 0.0
    rospy.loginfo("Stopping robot...")
    for _ in range(10):  # 1 seconde à 10 Hz
        pub.publish(stop_cmd)
        rospy.sleep(0.1)

def main():
    rospy.init_node('trajectory_terrain_varie')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)
    cmd = Twist()

    # Phase 1 : Avance droite (20 s)
    cmd.linear.x = 0.15
    cmd.angular.z = 0.0
    for _ in range(200):
        pub.publish(cmd)
        rate.sleep()

    # Phase 2 : Courbe à gauche (10 s)
    cmd.linear.x = 0.15
    cmd.angular.z = 0.3
    for _ in range(100):
        pub.publish(cmd)
        rate.sleep()

    # Phase 3 : Courbe à droite (10 s)
    cmd.linear.x = 0.15
    cmd.angular.z = -0.3
    for _ in range(100):
        pub.publish(cmd)
        rate.sleep()

    # Stop
    stop_robot(pub)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
