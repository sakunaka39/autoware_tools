cmake_minimum_required(VERSION 3.5)
project(planning_debug_tools)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rviz_common REQUIRED)

find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Core Qt5::Widgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
add_definitions(-DQT_NO_KEYWORDS)

# Extract version from package.xml
file(READ "${CMAKE_CURRENT_SOURCE_DIR}/package.xml" PACKAGE_XML_CONTENTS)
string(REGEX MATCH "<version>([0-9.]+)</version>" _ ${PACKAGE_XML_CONTENTS})
set(PACKAGE_VERSION "${CMAKE_MATCH_1}")
add_compile_definitions(PACKAGE_VERSION="${PACKAGE_VERSION}")

rosidl_generate_interfaces(
  planning_debug_tools
  "msg/TrajectoryDebugInfo.msg"
  "msg/PerceptionReproducerStatus.msg"
  DEPENDENCIES builtin_interfaces
)

ament_auto_add_library(trajectory_analyzer_node SHARED
  src/trajectory_analyzer.cpp
)

ament_auto_add_library(stop_reason_visualizer_node SHARED
  src/stop_reason_visualizer.cpp
)

if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
    rosidl_target_interfaces(trajectory_analyzer_node
    planning_debug_tools "rosidl_typesupport_cpp")
else()
    rosidl_get_typesupport_target(
            cpp_typesupport_target planning_debug_tools "rosidl_typesupport_cpp")
    target_link_libraries(trajectory_analyzer_node "${cpp_typesupport_target}")
endif()


rclcpp_components_register_node(trajectory_analyzer_node
  PLUGIN "planning_debug_tools::TrajectoryAnalyzerNode"
  EXECUTABLE trajectory_analyzer_exe
)

rclcpp_components_register_node(stop_reason_visualizer_node
  PLUGIN "planning_debug_tools::StopReasonVisualizerNode"
  EXECUTABLE stop_reason_visualizer_exe
)

ament_auto_add_executable(perception_replayer
  src/perception_replayer/time_manager_widget.cpp
  src/perception_replayer/perception_replayer_common.cpp
  src/perception_replayer/perception_replayer.cpp
  src/perception_replayer/perception_replayer_node.cpp
)

target_link_libraries(perception_replayer
  ${QT_LIBRARIES}
)

ament_auto_add_executable(perception_reproducer
  src/perception_replayer/perception_replayer_common.cpp
  src/perception_replayer/perception_reproducer.cpp
  src/perception_replayer/perception_reproducer_node.cpp
)

target_link_libraries(perception_reproducer
  ${QT_LIBRARIES}
)

# パネルライブラリのビルド
add_library(planning_debug_tools_rviz_plugins SHARED
  src/perception_reproducer_panel/perception_reproducer_panel.cpp
)
# MOC処理を有効化 (Qtのシグナル/スロットマクロ展開に必須)
set_target_properties(planning_debug_tools_rviz_plugins PROPERTIES AUTOMOC ON)

target_link_libraries(planning_debug_tools_rviz_plugins
  Qt5::Widgets
)

# 【修正】"planning_debug_tools" を削除
ament_target_dependencies(planning_debug_tools_rviz_plugins
  rclcpp
  rviz_common
)

# 【追加】自パッケージのカスタムメッセージを利用するためのリンク設定
if(${rosidl_cmake_VERSION} VERSION_LESS 2.5.0)
    rosidl_target_interfaces(planning_debug_tools_rviz_plugins
    planning_debug_tools "rosidl_typesupport_cpp")
else()
    rosidl_get_typesupport_target(
            cpp_typesupport_target planning_debug_tools "rosidl_typesupport_cpp")
    target_link_libraries(planning_debug_tools_rviz_plugins "${cpp_typesupport_target}")
endif()

pluginlib_export_plugin_description_file(rviz_common planning_debug_tools_plugins.xml)

install(TARGETS planning_debug_tools_rviz_plugins
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_auto_package(
  INSTALL_TO_SHARE
    launch
)

install(PROGRAMS
  scripts/processing_time_checker.py
  scripts/trajectory_visualizer.py
  scripts/closest_velocity_checker.py
  scripts/update_logger_level.sh
  DESTINATION lib/${PROJECT_NAME}
)
