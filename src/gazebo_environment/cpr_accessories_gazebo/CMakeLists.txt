cmake_minimum_required(VERSION 3.8)
project(cpr_accessories_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY meshes urdf launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
