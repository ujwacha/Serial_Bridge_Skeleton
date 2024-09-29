cmake_minimum_required(VERSION 3.8)
project(Serial_Bridge_Skeleton)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)

# set(SERIAL_LIB "/usr/local/lib/libserial.so")
# set(SERIAL_INCLUDE_DIR "/usr/local/include/libserial")

include_directories(${SERIAL_INCLUDE_DIR})

# Add include directories

#add_library(Serial_Skili_component SHARED src/main.cpp)
add_executable(Serial_Skili_component src/main.cpp)
#target_compile_definitions(Serial_Skili PRIVATE "COMPOSITION_BUILDING_DLL")

target_include_directories(Serial_Skili_component PUBLIC include/Serial_Bridge_Skeleton)

ament_target_dependencies(Serial_Skili_component rclcpp std_msgs geometry_msgs rclcpp_components )

target_link_libraries(Serial_Skili_component serial)


# rclcpp_components_register_node(
#   Serial_Skili_component 
#   PLUGIN "Serial_Bridge_Skeleton::Sayer"
#   EXECUTABLE Serial_Skili 
#   )



# ament_export_targets(export_Serial_Skili_component)



# install(TARGETS
#   Serial_Skili_component 
#   EXPORT export_Serial_Skili_component
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
#   )


install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
