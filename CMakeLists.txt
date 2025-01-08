cmake_minimum_required(VERSION 3.22)
project(interactive_marker_tutorials LANGUAGES CXX)

# Default to C99
if(NOT CMAKE_C_STANDARD)
 set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
 set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
 add_compile_options(-Wall -Wextra -Wpedantic -Werror=conversion -Werror=unused-but-set-variable
   -Werror=return-type -Werror=shadow -Werror=format -Werror=range-loop-construct
   -Werror=missing-braces)
endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rviz_common REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt5 REQUIRED COMPONENTS Core Widgets)
# find_package(Qt6 REQUIRED COMPONENTS Core Widgets)


# Automoc and moc((meta objects complier) file generation
set(CMAKE_AUTOMOC ON)
qt5_wrap_cpp(MOC_FILES
 include/interactive_marker_tutorials/basic_controls.hpp
)

# Add basic_controls as a shared library
add_library(basic_controls SHARED src/basic_controls.cpp ${MOC_FILES})


# Include the header directory
target_include_directories(basic_controls PUBLIC
 $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
 $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# include_directories($ENV{AMENT_PREFIX_PATH}/include)
include_directories(
 ${rviz_common_INCLUDE_DIRS}
)

# Link dependencies
ament_target_dependencies(basic_controls
 interactive_markers
 rclcpp
 rclcpp_components
 pluginlib
 rviz_common
 rviz_rendering
 tf2
 tf2_ros
 tf2_geometry_msgs
 geometry_msgs
 visualization_msgs
 Qt5Widgets
)
# Link Qt5 libraries
target_link_libraries(basic_controls Qt5::Widgets)


# Register the node as a component
rclcpp_components_register_nodes(basic_controls "interactive_marker_tutorials::BasicControlsNode")


# Install header files
install(DIRECTORY include/
 DESTINATION include/interactive_marker_tutorials
)
# Install the library
install(TARGETS basic_controls
 EXPORT export_interactive_marker_tutorials
 RUNTIME DESTINATION bin
 ARCHIVE DESTINATION lib
 LIBRARY DESTINATION lib
)


# Install plugin XML file
install(FILES basic_controls.xml
 DESTINATION share/${PROJECT_NAME}
)
# Export plugin description and directories
ament_export_include_directories(include)
ament_export_targets(export_interactive_marker_tutorials HAS_LIBRARY_TARGET)
pluginlib_export_plugin_description_file(rviz_common basic_controls.xml)


# Add standalone test executable
add_executable(basic_controls_test src/basic_controls_test.cpp)


# Link the test executable to necessary libraries
ament_target_dependencies(basic_controls_test
 rclcpp
 interactive_markers
 visualization_msgs
)
# Link against the basic_controls library
target_link_libraries(basic_controls_test basic_controls)


# Install the test executable
install(TARGETS basic_controls_test
 RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
 find_package(ament_lint_auto REQUIRED)
 ament_lint_auto_find_test_dependencies()
endif()

# Mark the package
ament_package()

#Mo