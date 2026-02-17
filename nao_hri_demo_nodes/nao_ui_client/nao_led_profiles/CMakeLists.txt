cmake_minimum_required(VERSION 3.8)
project(nao_led_profiles)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
