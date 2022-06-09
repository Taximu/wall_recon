cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
# set(ROS_BUILD_TYPE RelWithDebInfo)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DTIXML_USE_STL")

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(Boost REQUIRED COMPONENTS program_options regex)
include_directories(${Boost_INCLUDE_DIRS})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PCL 1.5 REQUIRED)

find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
rosbuild_add_boost_directories()

#TO CHANGE DEPENDENCY EDIT FILE (manifest.xml)

#=============================================================================================================#
#-------------------------------------------MAIN_BUILDS-------------------------------------------------------#
#=============================================================================================================#

#[DEPENDENCY  =  PCL_1.5]
rosbuild_add_executable(sac src/sac.cpp src/sac_configuration.cpp src/file_scan.cpp)
rosbuild_link_boost(sac thread signals system)
target_link_libraries(sac libtinyxml.so ${Boost_LIBRARIES})


#[DEPENDENCY  =  PCL_1.5]
rosbuild_add_executable(remove_inclination src/remove_inclination.cpp src/remove_inclination_configuration.cpp src/transform.cpp src/file_scan.cpp)
rosbuild_link_boost(remove_inclination thread signals system)
target_link_libraries(remove_inclination libtinyxml.so ${Boost_LIBRARIES})
target_link_libraries(remove_inclination ${OpenCV_LIBRARIES})


#[DEPENDENCY  =  PCL_1.5]
rosbuild_add_executable(identify_walls src/identify_walls.cpp src/identify_walls_configuration.cpp src/file_scan.cpp)
rosbuild_link_boost(identify_walls thread signals system)
target_link_libraries(identify_walls ${Boost_LIBRARIES})


#[DEPENDENCY  =  PCL_1.5]
rosbuild_add_executable(concatenate src/concatenate.cpp src/concatenate_configuration.cpp src/file_scan.cpp)
rosbuild_link_boost(concatenate thread signals system)
target_link_libraries(concatenate ${Boost_LIBRARIES})


#[DEPENDENCY  =  PCL_1.5]
rosbuild_add_executable(preparation_for_creating_image src/preparation_for_creating_image.cpp src/transform.cpp src/preparation_for_creating_image_configuration.cpp src/file_scan.cpp)
rosbuild_link_boost(preparation_for_creating_image thread signals system)
target_link_libraries(preparation_for_creating_image libtinyxml.so ${Boost_LIBRARIES})
target_link_libraries(preparation_for_creating_image ${OpenCV_LIBRARIES})


#[DEPENDENCY  =  PCL_1.5]
rosbuild_add_executable (orthogonal_projection src/orthogonal_projection.cpp src/orthogonal_projection_configuration.cpp src/file_scan.cpp src/transform.cpp)
rosbuild_link_boost (orthogonal_projection thread signals system)
target_link_libraries(orthogonal_projection libtinyxml.so ${Boost_LIBRARIES})
target_link_libraries(orthogonal_projection ${OpenCV_LIBRARIES})


#[DEPENDENCY  =  PCL_1.5]
rosbuild_add_executable(create_image src/create_image.cpp src/create_image_configuration.cpp src/EasyBMP.cpp src/file_scan.cpp)
rosbuild_link_boost(create_image thread signals system)
target_link_libraries(create_image libtinyxml.so ${Boost_LIBRARIES})
target_link_libraries(create_image ${OpenCV_LIBRARIES})


#[DEPENDENCY  =  PCL_1.5]
rosbuild_add_executable(model_environment src/model_environment.cpp)
rosbuild_link_boost(model_environment thread signals system)
target_link_libraries(model_environment libtinyxml.so ${Boost_LIBRARIES})


#=============================================================================================================#
#------------------------------------------EXTRA_BUILDS-------------------------------------------------------#
#=============================================================================================================#

#[DEPENDENCY  =  PCL_1.5]
#rosbuild_add_executable (concave_hull src/concave_hull.cpp src/concave_hull_configuration.cpp)
#rosbuild_link_boost (concave_hull thread signals system)
#target_link_libraries(concave_hull libtinyxml.so ${Boost_LIBRARIES})

#[DEPENDENCY  =  PCL_1.5]
#rosbuild_add_executable (radial_projection src/radial_projection.cpp src/radial_projection_configuration.cpp)
#rosbuild_link_boost (radial_projection thread signals system)
#target_link_libraries(radial_projection libtinyxml.so ${Boost_LIBRARIES})


#[DEPENDENCY  =  PCL_1.5]
#rosbuild_add_executable(voxel_grid src/voxel_grid.cpp src/voxel_grid_configuration.cpp)
#rosbuild_link_boost(voxel_grid thread signals system)
#target_link_libraries(voxel_grid libtinyxml.so ${Boost_LIBRARIES})


#[DEPENDENCY  =  PCL_1.5]
#rosbuild_add_executable(feasible_areas src/feasible_areas.cpp src/feasible_areas_configuration.cpp)
#rosbuild_link_boost(feasible_areas thread signals system)
#target_link_libraries(feasible_areas libtinyxml.so ${Boost_LIBRARIES})


#rosbuild_add_executable(test src/test.cpp)
#target_link_libraries(test libtinyxml.so ${Boost_LIBRARIES})
