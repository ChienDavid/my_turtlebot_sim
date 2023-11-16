#!/usr/bin/env python3

import math
import rospy
import rostest
import unittest
from gazebo_msgs.msg import ModelStates



class TestRobotMoves(unittest.TestCase):
    def movement_callback(self, data):
        """Get the robot's position"""
        # find the index of the turtlebot model within all Gazebo models
        if self.idx == 0:
            for name in data.name:
                if name == 'turtlebot3_burger':
                    break
                self.idx += 1
                
        # save current x,y position in the sim world
        if self.idx < len(data.pose):
            self.x = data.pose[self.idx].position.x
            self.y = data.pose[self.idx].position.y

    def get_distance_from_spawn(self):
        """Get total distance away from the spawn point"""
        return math.sqrt(abs(self.x)**2 + abs(self.y)**2)


    def test_movement(self):
        """Test that the robot is continuously movement"""
        self.idx = 0
        self.x = 0.
        self.y = 0.

        # set expected distance
        expected_travel = 1.5

        # Subscribe to Gazebo model states which contain the robot's position
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.movement_callback)

        # keep iterating until the robot travels as far as we expect
        travelled = self.get_distance_from_spawn()
        while travelled < expected_travel and not rospy.is_shutdown():
            rospy.loginfo("Travelled: {:.1f}m".format(travelled))
            rospy.sleep(1)
            travelled = self.get_distance_from_spawn()
        
        # when the robot has travelled as far as we expect, test complete.
        assert travelled >= expected_travel






if __name__=='__main__':
    rospy.init_node("tests", log_level=rospy.DEBUG)

    rostest.rosrun('my_turtlebot_sim', 'test_moves', TestRobotMoves)


