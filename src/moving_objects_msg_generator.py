#!/usr/bin/env python  

# Standard Python
from __future__ import division
import math
from math import isnan
from time import sleep
from time import localtime, strftime
from random import randint

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
        self.target_frame = "/{}/base_footprint".format(self.unique_name)
        self.state = Kalman(1, 0.2)
        self.subscriber = False
        self.route = []
        self.last_route_timestamp = rospy.Time().now()
        self.route_timeout = rospy.Duration.from_sec(5)
        # use only every X point in path, this safes alot of computing
        # power, good values may change with different configs
        self.route_skip_points = 25
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
        i = 0
        for p in msg.poses:
            i += 1
            if i % self.route_skip_points == 0:
                if not (isnan(p.pose.position.x) or isnan(p.pose.position.y) or isnan(p.pose.position.z) or \
                    isnan(p.pose.orientation.x) or isnan(p.pose.orientation.y) or isnan(p.pose.orientation.z) or isnan(p.pose.orientation.w)):
                    path.append(self.tflistener.transformPose(self.source_frame, p))
                else:
                    rospy.logerr("{}: Error in robot path for robot  {}".format(strftime("%d.%m.%Y %H:%M:%S", localtime()), self.unique_name))
                    return False
        
        #path = msg.poses # copy poses untransformed
        print path
        self.route = path
        self.last_route_timestamp = rospy.Time().now()
    
    def build_msg(self):
        # Get the guesed position
        try:
            #(trans,rot) = listener.lookupTransform(source_frame, target_frame.format(robot), rospy.Time.now())
            (trans,rot) = self.tflistener.lookupTransform(self.source_frame, self.target_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("{}: Error own position for robot {}".format(strftime("%d.%m.%Y %H:%M:%S", localtime()), self.unique_name))
            rospy.loginfo(e)
            return False
            
            
        # Feed Kalman filter with raw values
        raw_x, raw_y = trans[:2]
        raw_orientation = euler_from_quaternion(rot)[2]
        
        if not(isnan(raw_x) or isnan(raw_y) or isnan(raw_orientation)):
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
    refresh_rate = 4.0
    print "Init node"
    rospy.init_node('moving_objects_msg_generator')
    rate = rospy.Rate(refresh_rate)
    
    state_pub = rospy.Publisher('moving_objects', virtual_obstacles.msg.moving_object_msg, queue_size=4)
    
    robot_names = ["Turtlebot1", "Turtlebot2", "Turtlebot3", "Turtlebot4"]
    
    robots = [RobotState(name, refresh_rate) for name in robot_names]
    
    print "Start loop"
    while not rospy.is_shutdown():
        for robot in robots:
            msg = robot.build_msg()
            if msg:
                state_pub.publish(msg)
            else:
                rospy.logwarn("{}: msg_failed".format(strftime("%d.%m.%Y %H:%M:%S", localtime())))
        
        if randint(0,10) == 0: # WTF? ;-)
            print "alive."
        
        rate.sleep()

if __name__ == '__main__':
    main()
