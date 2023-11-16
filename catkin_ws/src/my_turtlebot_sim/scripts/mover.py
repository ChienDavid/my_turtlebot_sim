#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class TurtlebotSim:
    def __init__(self) -> None:
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.vel = Twist()
        self.rate = rospy.Rate(10)

    def move(self, linear_vel, angular_vel):
        """Move the robot continuously by sending velocity commands"""
        # move the robot
        self.vel.linear.x = linear_vel
        self.vel.angular.z = angular_vel
        while not rospy.is_shutdown():
            self.vel_publisher.publish(self.vel)
            self.rate.sleep()



if __name__=='__main__':
    try:
        rospy.init_node("mover")
        robot = TurtlebotSim()
        robot.move(0.5, 0.2)

    except rospy.ROSInterruptException:
        pass
