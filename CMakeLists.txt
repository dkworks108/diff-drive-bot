cmake_minimum_required(VERSION 3.8)
project(diff_drive_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install directories
install(
  DIRECTORY config launch urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

# Optional directories installed if present
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
  install(
    DIRECTORY scripts
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/maps")
  install(
    DIRECTORY maps
    DESTINATION share/${PROJECT_NAME}
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests")
  install(
    DIRECTORY tests
    DESTINATION share/${PROJECT_NAME}
  )
endif()

ament_package()

