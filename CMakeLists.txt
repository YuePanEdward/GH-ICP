cmake_minimum_required(VERSION 2.8)

project(CloudControlNet)

#SET(CMAKE_BUILD_TYPE "Debug")

SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# VTK
FIND_PACKAGE(VTK_COMPONENTS)

# PCL
FIND_PACKAGE(PCL REQUIRED)
INCLUDE_DIRECTORIES(${PCL_INCLUDE_DIRS})
LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})
ADD_DEFINITIONS(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

# libLas
FIND_PACKAGE(libLAS REQUIRED)
IF(libLAS_FOUND) 
MESSAGE("LibLAS is found:"${LIBLAS_INCLUDE_DIR}) 
INCLUDE_DIRECTORIES(${LIBLAS_INCLUDE_DIR})
LINK_DIRECTORIES(${LIBLAS_LIBS})
LINK_LIBRARIES("/usr/local/lib/liblas.so.2.4.0")
ENDIF(libLAS_FOUND)


# Eigen
INCLUDE_DIRECTORIES("usr/include/eigen3")

# g2o
#LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)  
#SET( G2O_ROOT /usr/local/include/g2o )
#FIND_PACKAGE(G2O REQUIRED) 
#IF(G2O_FOUND) 
#INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR}) 
#MESSAGE("G2O lib is found:"${G2O_INCLUDE_DIR}) 
#ENDIF(G2O_FOUND)
#SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse) 


INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/src)
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR}/include)

ADD_EXECUTABLE(pose_optimizer test/ghicp_reg.cpp)

# here link to pcl and glog
TARGET_LINK_LIBRARIES(pose_optimizer ${PCL_LIBRARIES} glog) 
#TARGET_LINK_LIBRARIES(pose_optimizer ${G2O_LIBS})


