#!/usr/bin/env python3
# encoding: utf-8

import numpy as np
from typing import List, Optional, Tuple
import scipy

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class ObstacleMovementDetector:
  def __init__(self):
    rospy.init_node('filter')

    self.prev_distance: Optional[float] = 0.0

    self.pub = rospy.Publisher("scan_filtered", LaserScan, queue_size=10)
    subscriber = rospy.Subscriber("scan", LaserScan, self.scan_callback)

  def scan_callback(self, msg: LaserScan) -> None:
    filtered_ranges = self.get_filter_data(msg)

    filtered_msg = LaserScan()
    filtered_msg.header = msg.header
    filtered_msg.angle_min = msg.angle_min
    filtered_msg.angle_max = msg.angle_max
    filtered_msg.angle_increment = msg.angle_increment
    filtered_msg.time_increment = msg.time_increment
    filtered_msg.scan_time = msg.scan_time
    filtered_msg.range_min = msg.range_min
    filtered_msg.range_max = msg.range_max
    filtered_msg.ranges = filtered_ranges

    self.pub.publish(filtered_msg)

  def get_filter_data(self, msg):
    if not msg.ranges:
        return []

    ranges = np.array(msg.ranges, dtype=np.float32)

    ranges[~np.isfinite(ranges)] = msg.range_max

    ranges = np.clip(ranges, msg.range_min, msg.range_max)

    window_size = 5
    kernel = np.ones(window_size) / window_size
    moving_avg = np.convolve(ranges, kernel, mode='same')

    median_filtered = scipy.signal.medfilt(moving_avg, kernel_size=5)

    median_filtered = np.clip(median_filtered, msg.range_min, msg.range_max)

    return median_filtered.tolist()

def main() -> None:
  detector = ObstacleMovementDetector()
  rospy.spin()

if __name__ == '__main__':
  main()
