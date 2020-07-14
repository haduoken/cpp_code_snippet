cmake_minimum_required(VERSION 3.5)
project(test_app)

set(CMAKE_CXX_STANDARD 11)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Eigen3)
find_package(G2O REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenCV REQUIRED QUIET)

message(STATUS         ${PCL_INCLUDE_DIRS})
message(STATUS                ${G2O_INCLUDE_DIR})
INCLUDE_DIRECTORIES(
        ${EIGEN3_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        "/opt/ros/kinetic/include"
        )

add_executable(test_app main.cpp)
target_link_libraries(test_app ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )

add_executable(test_icp pcl/test_icp.cpp)
target_link_libraries(test_icp ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )

add_executable(icp_with_trans pcl/icp_with_trans.cpp)
target_link_libraries(icp_with_trans ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )

add_executable(test_iteration eigen/t_iteration.cpp)
target_link_libraries(test_iteration ${G2O_LIBRARIES} ${Sophus_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_iteration1 eigen/t_iteration1.cpp)
target_link_libraries(test_iteration1 ${G2O_LIBRARIES} ${Sophus_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_iteration2 g2o/t_iteration2.cpp)
target_link_libraries(test_iteration2 ${G2O_LIBRARIES} ${Sophus_LIBRARIES} ${OpenCV_LIBRARIES})


add_executable(01_basic_usage eigen/01_basic_usage.cpp)
target_link_libraries(01_basic_usage ${G2O_LIBRARIES})

add_executable(02_block eigen/02_block.cpp)
target_link_libraries(02_block ${G2O_LIBRARIES})