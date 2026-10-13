cmake_minimum_required(VERSION 3.5)
project(practica3)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generar servicios
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GoToPose.srv"
)

ament_package()
