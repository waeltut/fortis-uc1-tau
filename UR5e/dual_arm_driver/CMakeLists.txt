cmake_minimum_required(VERSION 3.19)
project(dual_arm_driver)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})
# Symlink installs execute the source scripts directly. Restore executable
# permissions during configuration, including after ZIP/Windows extraction.
file(CHMOD
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/tcp_pose_publisher"
  "${CMAKE_CURRENT_SOURCE_DIR}/scripts/payload_initializer"
  PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE
              GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE)
install(PROGRAMS scripts/tcp_pose_publisher scripts/payload_initializer
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
