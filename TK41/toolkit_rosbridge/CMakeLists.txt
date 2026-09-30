cmake_minimum_required(VERSION 3.8)
project(toolkit_rosbridge)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch config docs examples DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE)
install(FILES README.md LICENSE DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/demo_node.py DESTINATION lib/${PROJECT_NAME}
  RENAME demo_node)
ament_package()
