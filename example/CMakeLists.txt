cmake_minimum_required(VERSION 3.16)
project(rfid_example C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

add_executable(rfid_test main.c)

# ===== include paths =====
target_include_directories(rfid_test PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/../include
  ${CMAKE_CURRENT_LIST_DIR}/../third_party/mercuryapi/c/include
)

# ===== so library path =====
target_link_directories(rfid_test PRIVATE
  ${CMAKE_CURRENT_LIST_DIR}/../build
)

# ===== link against libMercuryApi.so =====
# libMercuryApi.so -> MercuryApi
target_link_libraries(rfid_test PRIVATE
  MercuryApi
)

# ===== runtime search path (no LD_LIBRARY_PATH needed) =====
set_target_properties(rfid_test PROPERTIES
  BUILD_RPATH "${CMAKE_CURRENT_LIST_DIR}/../build"
)

