#!/usr/bin/env python3
# encoding: utf-8
from visualization_msgs.msg import Marker
import numpy as np
from typing import List, Optional, Tuple
import scipy
from geometry_msgs.msg import Point
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32, String
from geometry_msgs.msg import Twist

class ObstacleMovementDetector:
  def __init__(self):
    rospy.init_node('movement_detector')

    self.prev_distance: Optional[float] = None

    self.pub_dist = rospy.Publisher("wall_distance", Float32, queue_size=10)
    self.pub_mot = rospy.Publisher("wall_motion", String, queue_size=10)
    self.marker_pub = rospy.Publisher("visualization_marker", Marker, queue_size=10)
    self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    self.prev_error = 0.0
    self.integral = 0.0

    subscriber = rospy.Subscriber("scan_filtered", LaserScan, self.scan_callback)

  def scan_callback(self, msg: LaserScan) -> None:
    self.movement_detection(msg)


  def draw_motion_arrow(self, msg: LaserScan, length: float) -> None:
    marker = Marker()
    marker.header.frame_id = msg.header.frame_id
    marker.header.stamp = rospy.Time.now()
    marker.ns = "motion_arrow"
    marker.id = 0
    marker.type = Marker.ARROW
    marker.action = Marker.ADD

    start_point = Point()
    start_point.x = 0.0
    start_point.y = 0.0
    start_point.z = 0.0

    center_angle = msg.angle_min + (msg.angle_max - msg.angle_min) / 2
    end_point = Point()
    end_point.x = length * np.cos(center_angle)
    end_point.y = length * np.sin(center_angle)
    end_point.z = 0.0

    marker.points.append(start_point)
    marker.points.append(end_point)

    marker.scale.x = 0.05  
    marker.scale.y = 0.1   
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0
    marker.lifetime = rospy.Duration(1)  

    self.marker_pub.publish(marker)

  def pid(self, current_distance):
    target_distance = 1

    Kp = 2.0   
    Kd = 0.005   
    Ki = 0.4

    MAX_SPEED = 1.6 

    dt = 0.1  


    error = target_distance - current_distance

    P = Kp * error

    D = Kd * (error - self.prev_error) / dt

    self.integral += error * dt
    I = Ki * self.integral

    control_signal = P + D + I

    control_signal = -max(-MAX_SPEED, min(control_signal, MAX_SPEED))

    self.prev_error = error

    twist = Twist()
    twist.linear.x = control_signal  
    twist.angular.z = 0.0            
    print(control_signal)
    self.cmd_vel_pub.publish(twist)

  def movement_detection(self, msg):
    frontal_ranges = []

    minIndex = (int(abs(msg.angle_max-2.5*3.14/180)/msg.angle_increment)) #298
    maxIndex = (int(abs(msg.angle_min-2.5*3.14/180)/msg.angle_increment)) #386

    currIndex = minIndex + 1
    while currIndex <= maxIndex:
        frontal_ranges.append(msg.ranges[currIndex])
    # print(minIndex, " ", maxIndex)
    
    current_distance = msg.ranges[maxIndex]

    dist_msg = Float32()
    dist_msg.data = current_distance
    self.pub_dist.publish(dist_msg)

    motion_state = ""

    if self.prev_distance is not None:
        delta = current_distance - self.prev_distance
        if abs(delta) <= 0.03:  # 3 см
            motion_state = "STABLE"
            self.draw_motion_arrow(msg, 0.0)  
        elif delta > 0:
            motion_state = "RECEDING"
            self.draw_motion_arrow(msg, -0.5)  
        else:
            motion_state = "APPROACHING"
            self.draw_motion_arrow(msg, 0.5)   
    else:
        motion_state = "STABLE"

    self.prev_distance = current_distance

    mot_msg = String()
    mot_msg.data = motion_state
    self.pub_mot.publish(mot_msg)

    self.pid(current_distance)



def main() -> None:
  detector = ObstacleMovementDetector()
  rospy.spin()

if __name__ == '__main__':
  main()
