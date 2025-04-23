cmake_minimum_required(VERSION 3.8)
project(extended_camera_loader)
#set(CMAKE_POSITION_INDEPENDENT_CODE ON)
## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
set(DEPENDENCIES
  rclcpp
  image_transport
  extended_camera_model
)
foreach(Dependency IN ITEMS ${DEPENDENCIES})
  find_package(${Dependency} REQUIRED)
endforeach()


## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  include/${PROJECT_NAME}/camera_loader.h
  include/${PROJECT_NAME}/camera.h
  src/camera_loader.cpp
  src/camera.cpp
)
target_include_directories(${PROJECT_NAME}
        PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
        )
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

# Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES} Eigen3)

ament_package()
