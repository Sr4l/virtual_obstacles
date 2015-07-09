#!/usr/bin/env python  

# Standard Python
from __future__ import division
import math
from time import sleep

# ROS packages
import roslib
roslib.load_manifest('virtual_obstacles')
import rospy
import tf

from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion

# own ROS packages
import virtual_obstacles.msg

# package intern imports
from Kalman import Kalman


class RobotState(object):
    def __init__(self, unique_name, refresh_rate):
        self.unique_name = unique_name
        self.source_frame = "/map"
        self.target_frame = "/{}/base_link".format(self.unique_name)
        self.state = Kalman(1, 0.2)
        self.subscriber = False
        self.route = []
        self.last_route_timestamp = rospy.Time().now()
        self.route_timeout = rospy.Duration.from_sec(5)
        self.refresh_rate = refresh_rate
        
        self.tflistener = tf.TransformListener()
    
        self.subscriber = rospy.Subscriber(
            "{}/move_base/DWAPlannerROS/global_plan".format(self.unique_name),
            Path, self.handle_robot_path)
    
    def handle_robot_path(self, msg):
        """
        receive routes and save them transformed to map_frame
        """
        path = []
        for p in msg.poses:
            path.append(self.tflistener.transformPose(self.source_frame, p))
        
        #path = msg.poses # copy poses untransformed
        self.route = path
        self.last_route_timestamp = rospy.Time().now()
    
    def build_msg(self):
        # Get the guesed position
        try:
            #(trans,rot) = listener.lookupTransform(source_frame, target_frame.format(robot), rospy.Time.now())
            (trans,rot) = self.tflistener.lookupTransform(self.source_frame, self.target_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #print "Error for robot {}".format(robot)
            return False
            
        # Feed Kalman filter with raw values
        raw_x, raw_y = trans[:2]
        raw_orientation = euler_from_quaternion(rot)[2]
        
        self.state.update([raw_x, raw_y, raw_orientation], 1.0/self.refresh_rate)
        
        # Get position, orientation and velocitys from Kalman filter
        x, y, yaw = self.state.position().tolist()[0]
        vx, vy, vyaw = self.state.velocity().tolist()[0]
        
        # Build and publish message
        msg = virtual_obstacles.msg.moving_object_msg()
        msg.unique_name = self.unique_name
        msg.pose.x = x
        msg.pose.y = y
        msg.pose.theta = yaw
        
        msg.velocity.linear.x = vx
        msg.velocity.linear.y = vy
        msg.velocity.angular.z = vyaw
        
        if (rospy.Time().now() - self.last_route_timestamp) <= self.route_timeout:
            msg.route = self.route
        else:
            msg.route = []
        
        return msg

def main():
    refresh_rate = 10.0
    print "Init node"
    rospy.init_node('moving_objects_msg_generator')
    rate = rospy.Rate(refresh_rate)
    
    state_pub = rospy.Publisher('moving_objects', virtual_obstacles.msg.moving_object_msg, queue_size=10)
    
    robot_names = ["Turtlebot1", "Turtlebot2", "Turtlebot3", "Turtlebot4"]
    
    robots = [RobotState(name, refresh_rate) for name in robot_names]
    
    print "Start loop"
    while not rospy.is_shutdown():
        for robot in robots:
            msg = robot.build_msg()
            if msg:
                state_pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    main()
