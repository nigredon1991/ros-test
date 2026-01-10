# Файлы для робота

```sh
# Build
./build-docker.sh

# Run container
./docker-run.sh

# Exec to container
# First time
docker exec -it ros /ros_entrypoint.sh bash
```


**docker-compose.yaml**
```yml
version: '2'

services:
  talker:
    image: osrf/ros:jazzy-desktop
    command: ros2 run demo_nodes_cpp talker
  listener:
    image: osrf/ros:jazzy-desktop
    command: ros2 run demo_nodes_cpp listener
    depends_on:
      - talker
```

* `ros2 topic list` – показать все существующие топики;
* `ros2 topic pub` – ручная публикация сообщений;
* `ros2 topic echo` – «эхо», то есть прослушивание топика в реальном времени;
* `ros2 topic info` – информация о топике (тип сообщения, издатели, подписчики и пр.);
* `ros2 node list` – список активных нод в системе;
* `rqt_graph` – визуальное представление графа вычислений (какие ноды запущены и как они соединены топиками).

### Заметки

```sh
ros2 topic pub /chatter std_msgs/msg/String "{data: 'Hello, ROS2'}"
ros2 topic echo /chatter
ros2 run turtlesim turtlesim_node
ros2 run my_turtle_controller turtle_mover
# packages
cd src
ros2 pkg create --build-type ament_python talker
ros2 pkg create --build-type ament_python listener
# create packages
cd ..
colcon build
colcon build --packages-select talker

# Сервисы
ros2 service list
ros2 service type /spawn
ros2 interface show turtlesim/srv/Spawn
ros2 service call /spawn turtlesim/srv/Spawn "{x: 2.0, y: 5.0, theta: 0.0, name: 'my_turtle'}"

ros2 pkg create my_service_interfaces
ros2 run my_service_server velocity_server
 ros2 service call /set_velocity my_service_interfaces/srv/SetVelocity "{linear: 2.0, angular: 5.0}"
```
