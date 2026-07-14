cmake_minimum_required(VERSION 3.16)
project(foxy_gz_plugins VERSION 0.3.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(gz-cmake3 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 8.14 REQUIRED)
find_package(gz-sensors8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-math7 REQUIRED)

add_library(foxy_gz_led_system SHARED
  src/led_system.cc
)

add_library(${PROJECT_NAME}::foxy_gz_led_system ALIAS foxy_gz_led_system)

target_include_directories(foxy_gz_led_system
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(foxy_gz_led_system
  PUBLIC
    gz-sim8::gz-sim8
    gz-sensors8::gz-sensors8
    gz-plugin2::gz-plugin2
    gz-transport13::gz-transport13
    gz-msgs10::gz-msgs10
    gz-math7::gz-math7
)

# Keep the library filename simple for SDF:
#   <plugin filename="foxy_gz_led_system" name="foxy_gz_plugins::LedSystem"/>
set_target_properties(foxy_gz_led_system PROPERTIES
  OUTPUT_NAME "foxy_gz_led_system"
)

install(
  TARGETS foxy_gz_led_system
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY examples/
  DESTINATION share/${PROJECT_NAME}/examples
)

# Make `source install/setup.bash` expose this package's plugins to Gazebo Sim.
# The .dsv hook is shell-agnostic and gets expanded relative to AMENT_CURRENT_PREFIX.
ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/foxy_gz_plugins.dsv.in"
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(
  gz-cmake3
  gz-plugin2
  gz-sim8
  gz-sensors8
  gz-transport13
  gz-msgs10
  gz-math7
)

ament_package()
