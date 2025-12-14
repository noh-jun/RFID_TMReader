cmake_minimum_required(VERSION 3.22)
project(MercuryApi LANGUAGES C VERSION 1.31.4)

set(CMAKE_C_STANDARD 11)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(MERCURY_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/mercuryapi/c)
set(MERCURY_SRC_DIR ${MERCURY_ROOT}/src)
set(MERCURY_INC_DIR ${MERCURY_ROOT}/include)

file(GLOB MERCURY_SOURCES CONFIGURE_DEPENDS
  ${MERCURY_SRC_DIR}/*.c
)

add_library(MercuryApi SHARED
  ${MERCURY_SOURCES}
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mercuryapi_wrapper.c
  ${CMAKE_CURRENT_SOURCE_DIR}/src/mercuryapi_version.c
)

# 외부 include는 통합 헤더 1개만
target_include_directories(MercuryApi
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${MERCURY_INC_DIR}
)

target_link_libraries(MercuryApi PUBLIC pthread m)

set_target_properties(MercuryApi PROPERTIES
  OUTPUT_NAME "MercuryApi"
  VERSION ${PROJECT_VERSION}
  SOVERSION 1
)

install(TARGETS MercuryApi LIBRARY DESTINATION lib)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/include/mercuryapi.h DESTINATION include)
