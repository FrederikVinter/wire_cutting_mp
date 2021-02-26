cmake_minimum_required(VERSION 2.8.3)
project(wire_cutting)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  tesseract_monitoring
  tesseract_rosutils
  tesseract_planning_server
  octomap_ros
  pcl_conversions
)

find_package(tesseract_environment REQUIRED)
find_package(tesseract_command_language REQUIRED)
find_package(tesseract_motion_planners REQUIRED)
find_package(tesseract_process_managers REQUIRED)
find_package(tesseract_common REQUIRED)
find_package(trajopt_sqp REQUIRED)
find_package(trajopt_ifopt REQUIRED)
find_package(ros_industrial_cmake_boilerplate REQUIRED)
find_package(PCL REQUIRED COMPONENTS core features filters io segmentation surface)

catkin_package()

# Load variable for clang tidy args, compiler options and cxx version
tesseract_variables()


# WIRECUTTING:
add_library(${PROJECT_NAME}_wire_cutting src/wire_cutting.cpp 
                                         src/trajopt_wire_cutting_plan_profile.cpp)
                                         

target_link_libraries(${PROJECT_NAME}_wire_cutting tesseract::tesseract_environment_core tesseract::tesseract_environment_ofkt tesseract::tesseract_process_managers ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_wire_cutting PUBLIC ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_wire_cutting ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_wire_cutting PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_wire_cutting PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_wire_cutting SYSTEM PUBLIC
    ${catkin_INCLUDE_DIRS})


add_executable(${PROJECT_NAME}_wire_cutting_node src/wire_cutting_node.cpp)
target_link_libraries(${PROJECT_NAME}_wire_cutting_node ${PROJECT_NAME}_wire_cutting ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_wire_cutting_node PRIVATE ${TESSERACT_COMPILE_OPTIONS})
target_clang_tidy(${PROJECT_NAME}_wire_cutting_node ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
target_cxx_version(${PROJECT_NAME}_wire_cutting_node PUBLIC VERSION 17)
target_include_directories(${PROJECT_NAME}_wire_cutting_node PRIVATE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_wire_cutting_node SYSTEM PRIVATE
    ${catkin_INCLUDE_DIRS})


# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
 )

# Mark executables and/or libraries for installation
install(
  TARGETS
    ${PROJECT_NAME}_wire_cutting
    ${PROJECT_NAME}_wire_cutting_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## OMPL Constraint planning is not available in version less than 1.4
#if(NOT OMPL_VERSION VERSION_LESS "1.4.0")
#  install(
#    TARGETS
#      ${PROJECT_NAME}_glass_upright_ompl_example
#      ${PROJECT_NAME}_glass_upright_ompl_example_node
#    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#  )
#endif()

install(
  DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  DIRECTORY test
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.test"
)

#[[if(CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    find_package(GTest REQUIRED)

    add_rostest_gtest(${PROJECT_NAME}_basic_cartesian_example_unit test/basic_cartesian_example_unit.test test/basic_cartesian_example_unit.cpp)
    target_link_libraries(${PROJECT_NAME}_basic_cartesian_example_unit GTest::GTest GTest::Main ${PROJECT_NAME}_basic_cartesian_example ${catkin_LIBRARIES})
    target_compile_options(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE ${TESSERACT_COMPILE_OPTIONS})
    target_clang_tidy(${PROJECT_NAME}_basic_cartesian_example_unit ARGUMENTS ${TESSERACT_CLANG_TIDY_ARGS} ENABLE ${TESSERACT_ENABLE_CLANG_TIDY})
    target_cxx_version(${PROJECT_NAME}_basic_cartesian_example_unit PUBLIC VERSION 17)
    target_include_directories(${PROJECT_NAME}_basic_cartesian_example_unit PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>")
    target_include_directories(${PROJECT_NAME}_basic_cartesian_example_unit SYSTEM PRIVATE
        ${catkin_INCLUDE_DIRS})
endif()]]

