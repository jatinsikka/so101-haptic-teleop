cmake_minimum_required(VERSION 3.16)
project(so101_description NONE)   # data only: no compiler needed

find_package(ament_cmake REQUIRED)

# Everything lands in install/share/so101_description/..., which is what
# package://so101_description/... and get_package_share_directory() resolve to.
install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME})

ament_package()
