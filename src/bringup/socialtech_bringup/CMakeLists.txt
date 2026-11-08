cmake_minimum_required(VERSION 3.8)
project(socialtech_bringup)
find_package(ament_cmake REQUIRED)
# launch/ and config/ are found through the package share directory; scripts/ is installed flat under lib/ (they import each other and are run by path, never through `ros2 run`:
# a signal sent to `ros2 run` does not reach the program it starts, and the recorder in run_live needs its SIGINT).
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY scripts/ DESTINATION lib/${PROJECT_NAME} USE_SOURCE_PERMISSIONS)
ament_package()
