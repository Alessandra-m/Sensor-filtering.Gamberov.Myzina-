=======
**Задание 03. Sensor filtering** - *Гамберов Т. и Мызина А.*

Как бы вы реализовали оценку точности вычисления расстояния вашим алгоритмом в Gazebo?

Ответ: использовали бы api Gazebo для этой задачи

ros lidar:

rosrun ros_lidar_examples ros_lidar_1.py

rosrun ros_lidar_examples ros_lidar_2.py

sensor filtering: 

rosrun ros_lidar_examples filter_sensor_filtering.py

rosrun ros_lidar_examples movement_detection_sensor_filtering.py

pid:

rosrun ros_lidar_examples filter_pid.py

rosrun ros_lidar_examples movement_detection_pid.py


