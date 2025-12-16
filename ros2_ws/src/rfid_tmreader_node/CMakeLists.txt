cmake_minimum_required(VERSION 3.16)
project(rfid_tmreader_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(rfid_tmreader_node src/rfid_tmreader_node.cpp)

ament_target_dependencies(rfid_tmreader_node
  rclcpp
  std_msgs
)

target_include_directories(rfid_tmreader_node PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/../../../include
  ${CMAKE_CURRENT_LIST_DIR}/../../../third_party/mercuryapi/c/include
)

target_link_directories(rfid_tmreader_node PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/../../../build
)

# ✅ plain signature로 통일 (Jazzy의 ament_target_dependencies와 일치)
target_link_libraries(rfid_tmreader_node
  MercuryApi
)

set_target_properties(rfid_tmreader_node PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_LIST_DIR}/../../../build"
)

install(TARGETS rfid_tmreader_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
