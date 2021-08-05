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
find_package(Ceres REQUIRED)

message(STATUS         ${PCL_INCLUDE_DIRS})
message(STATUS                ${G2O_INCLUDE_DIR})
message(WARNING "Eigen Version": ${EIGEN_WORLD_VERSION}.${EIGEN_MAJOR_VERSION}.${EIGEN_MINOR_VERSION})
INCLUDE_DIRECTORIES(
        ${EIGEN3_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${Sophus_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        "/opt/ros/kinetic/include"
        ${CERES_INCLUDE_DIRS}
        )

add_executable(test_app main.cpp)
target_link_libraries(test_app ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )

add_executable(01_hash_map cpp_usage/01_hash_map.cpp)
add_executable(02_forward_as_tuple cpp_usage/02_forward_as_tuple.cpp)

# ********************************************************************
# ************************          GTSAM           ******************
# ********************************************************************
add_executable(01_gtsam gtsam/01_gtsam.cpp)
target_link_libraries(01_gtsam ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )
add_executable(02_gtsam gtsam/02_gtsam.cpp)
target_link_libraries(02_gtsam ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )
add_executable(03_pose3_estimate gtsam/03_pose3_estimate.cpp)
target_link_libraries(03_pose3_estimate ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )
add_executable(04_test_prior gtsam/04_test_prior.cpp)
target_link_libraries(04_test_prior ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )
add_executable(05_optimize_odom_and_prior gtsam/05_optimize_odom_and_prior.cpp)
target_link_libraries(05_optimize_odom_and_prior ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )
add_executable(06_test_noise_meaning gtsam/06_test_noise_meaning.cpp)
target_link_libraries(06_test_noise_meaning ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )
add_executable(07_test_gps_fusion gtsam/07_test_gps_fusion.cpp)
target_link_libraries(07_test_gps_fusion ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )
add_executable(08_custom_factor gtsam/08_custom_factor.cpp)
target_link_libraries(08_custom_factor ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )
add_executable(09_between_rotation gtsam/09_between_rotation.cpp)
target_link_libraries(09_between_rotation ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )
add_executable(10_compose_factor gtsam/10_compose_factor.cpp)
target_link_libraries(10_compose_factor ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )
add_executable(11_limitz_factor gtsam/11_limitz_factor.cpp)
target_link_libraries(11_limitz_factor ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )

# ********************************************************************
# ************************          PCL           ******************
# ********************************************************************
add_executable(test_icp pcl/test_icp.cpp)
target_link_libraries(test_icp ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )
add_executable(icp_with_trans pcl/icp_with_trans.cpp)
target_link_libraries(icp_with_trans ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )
add_executable(new_point_type pcl/new_point_type.cpp)
target_link_libraries(new_point_type ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam )



add_executable(test_iteration eigen/t_iteration.cpp)
target_link_libraries(test_iteration ${G2O_LIBRARIES} ${Sophus_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_iteration1 eigen/t_iteration1.cpp)
target_link_libraries(test_iteration1 ${G2O_LIBRARIES} ${Sophus_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_iteration2 g2o/t_iteration_g2o.cpp)
target_link_libraries(test_iteration2 ${G2O_LIBRARIES} ${Sophus_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(test_residual ceres/04_test_residual.cpp)
target_link_libraries(test_residual ${CERES_LIBRARIES})

add_executable(fit_plane ceres/05_fit_plane.cpp ceres/eigenpose.cpp)
target_link_libraries(fit_plane ${CERES_LIBRARIES} ${PCL_LIBRARIES})

add_executable(06_relative_pose ceres/06_relative_pose.cpp ceres/eigenpose.cpp)
target_link_libraries(06_relative_pose ${CERES_LIBRARIES} ${PCL_LIBRARIES})

add_executable(07_fit_rotation ceres/07_fit_rotation.cpp ceres/eigenpose.cpp)
target_link_libraries(07_fit_rotation ${CERES_LIBRARIES} ${PCL_LIBRARIES})

add_executable(08_icp_rotation ceres/08_icp_rotation.cpp ceres/eigenpose.cpp)
target_link_libraries(08_icp_rotation ${CERES_LIBRARIES} ${PCL_LIBRARIES})

add_executable(01_basic_usage eigen/01_basic_usage.cpp)
target_link_libraries(01_basic_usage ${G2O_LIBRARIES})

add_executable(02_block eigen/02_block.cpp)
target_link_libraries(02_block ${G2O_LIBRARIES})

add_executable(03_test_pose eigen/03_test_pose.cpp)
add_executable(t_iteration2 eigen/t_iteration2.cpp)


add_executable(03_use_deque cpp_usage/03_use_deque.cpp)
add_executable(04_map_find cpp_usage/04_map_find.cpp)
add_executable(05_vector_remove cpp_usage/05_vector_remove.cpp)
add_executable(06_heap_usage cpp_usage/06_heap_usage.cpp)
add_executable(07_union_usage  cpp_usage/07_union_usage.cpp)
add_executable(stl_test  app/stl_test.cpp)

add_executable(pcl_deque pcl/pcl_deque.cpp)
target_link_libraries(pcl_deque ${PCL_LIBRARIES} )

add_executable(operator_overload cpp_usage/operator_overload.cpp)
add_executable(parse_png_head cpp_usage/parse_png_head.cpp)
