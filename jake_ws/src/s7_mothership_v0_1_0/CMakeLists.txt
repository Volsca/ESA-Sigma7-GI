cmake_minimum_required(VERSION 3.8)
project(s7_mothership_v0_1_0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# add executables
add_executable(s7_mothership_node src/S7Mothership.cpp)

# include directories + drd & dhd
target_include_directories(s7_mothership_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/include>
  $<INSTALL_INTERFACE:include>
)

# link .so files
target_link_directories(s7_mothership_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party/libs
)

# link libraries
target_link_libraries(s7_mothership_node
  dhd
  drd
)

set_target_properties(s7_mothership_node PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/third_party/libs"
)

# setup dependencies
ament_target_dependencies(s7_mothership_node
  rclcpp
  geometry_msgs
  )

# install the executable/headers/3rd-party headers and libs
install(TARGETS s7_mothership_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY third_party/include/
  DESTINATION include
)
install(DIRECTORY third_party/libs/
  DESTINATION lib
)

#set debugging
#target_compile_definitions(s7_mothership_node PRIVATE DEBUG_OPERATION_TIME)
target_compile_definitions(s7_mothership_node PRIVATE DEBUG_RESPONSE_TIME)
#target_compile_definitions(s7_mothership_node PRIVATE DEBUG_LOCK_TIME)

ament_package()
