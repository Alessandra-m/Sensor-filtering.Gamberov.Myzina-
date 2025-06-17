# Домашнее задание для курса датчиков(3, 4, 5)
*Гамберов Т. и Мызина А.*


## №3 ros lidar:
```
rosrun sensors_hw fake_scan_publisher.py

rosrun sensors_hw ros_lidar_1.py

rosrun sensors_hw ros_lidar_2.py
```
## №4 sensor filtering:
```
roslaunch sensors_hw gazebo_world.launch

rosrun sensors_hw filter_sensor_filtering.py

rosrun sensors_hw movement_detection_sensor_filtering.py
```
## №5 pid:
```
roslaunch sensors_hw gazebo_world.launch

rosrun sensors_hw filter_pid.py

rosrun sensors_hw movement_detection_pid.py
```

Все скринкасты в папке **videos**


Как бы вы реализовали оценку точности вычисления расстояния вашим алгоритмом в Gazebo?
Ответ: использовали бы api Gazebo для этой задачи
