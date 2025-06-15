#!/usr/bin/env python3
# encoding: utf-8

import numpy as np
from typing import List, Optional, Tuple

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class ObstacleMovementDetector:
  def __init__(self):
    rospy.init_node('obstacle_movement_detector')

    # Параметры
    self.prev_distance: Optional[float] = 0.0

    self.pub = rospy.Publisher("obstacle_movement", String)
    subscriber = rospy.Subscriber("scan", LaserScan, self.scan_callback)

  def scan_callback(self, msg: LaserScan) -> None:
    start_t = rospy.get_time()

    frontal_ranges = self.get_frontal_sector(msg.ranges, msg.angle_min, msg.angle_max, msg.angle_increment)
    if not frontal_ranges:
      rospy.logwarn("Не удалось получить сканы фронального сектора!")
      return
    
    
    min_distance = np.min(frontal_ranges)

    movement, delta = self.detect_movement(min_distance)
    msg_str = String()
    msg_str.data = movement
    self.pub.publish(msg_str)

    print("speed: ", delta/(rospy.get_time-start_t))


  def get_frontal_sector(self, ranges: List[float], angle_min: float, angle_max: float, angle_increment: float) -> List[float]:
    """
    Возвращает список расстояний в заданном секторе.
    Исключает значения inf (бесконечности) и nan.
    """
    frontal_ranges = []

    minIndex = (int(abs(angle_max-15*3.14/180)/angle_increment)) #298
    maxIndex = (int(abs(angle_min-15*3.14/180)/angle_increment)) #386

    currIndex = minIndex + 1
    while currIndex <= maxIndex:
      if ranges[currIndex] < 0.5:
        print("Пороговое расстояние!!!")
        frontal_ranges.append(ranges[currIndex])
      currIndex = currIndex + 1

    return frontal_ranges

  def detect_movement(self, current_distance: float) -> Tuple[str, float]:
    """
    Определяет тип движения относительно препятствия.
    Возвращает тип движения и изменение расстояния.
    """
    movement = "stable"
    delta = 0.0

    if self.prev_distance is not None:
      delta = self.prev_distance - current_distance
      if delta > 0.02:
        if delta < 0:
          movement = "receding"
        elif delta > 0:
          movement = "approaching"

    self.prev_distance = current_distance
    return movement, delta


def main() -> None:
  detector = ObstacleMovementDetector()
  rospy.spin()

if __name__ == '__main__':
  main()
