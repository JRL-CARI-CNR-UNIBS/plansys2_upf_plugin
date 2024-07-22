cmake_minimum_required(VERSION 3.8)
project(plansys2_upf_plan_solver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(plansys2_core REQUIRED)
find_package(pluginlib REQUIRED)

set(dependencies
  rclcpp
  ament_index_cpp
  plansys2_core
  pluginlib
)

include_directories(
  include
)

add_library(${PROJECT_NAME} SHARED
  src/plansys2_upf_plan_solver/upf_plan_solver.cpp
)
ament_target_dependencies(${PROJECT_NAME} ${dependencies})

pluginlib_export_plugin_description_file(plansys2_core plansys2_upf_plan_solver_plugin.xml)

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES plansys2_upf_plan_solver_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
