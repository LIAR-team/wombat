cmake_minimum_required(VERSION 3.5)
project(wombat_srrg)

find_package(wombat_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)


set(CMAKE_MODULE_PATH /usr/local/lib/cmake/SuiteSparse)
message(WARNING "PATH ${CMAKE_MODULE_PATH}")
find_package(CHOLMOD REQUIRED)
find_package(UMFPACK REQUIRED)
find_package(CXSparse REQUIRED)

set(CHOLMOD_INCLUDE_DIRS ${CHOLMOD_INCLUDE_DIR})
set(UMFPACK_INCLUDE_DIRS ${UMFPACK_INCLUDE_DIR})
set(CXSPARSE_LIBRARIES ${CXSPARSE_LIBRARY})
set(CXSPARSE_INCLUDE_DIRS ${CXSPARSE_INCLUDE_DIR})

message(WARNING "CHOLMOD LIB ${CHOLMOD_LIBRARIES}")
message(WARNING "CHOLMOD INC ${CHOLMOD_INCLUDE_DIRS}")
message(WARNING "UMFPACK LIB ${UMFPACK_LIBRARIES}")
message(WARNING "UMFPACK INC ${UMFPACK_INCLUDE_DIRS}")
message(WARNING "CXSparse LIB ${CXSPARSE_LIBRARIES}")
message(WARNING "CXSparse INC ${CXSPARSE_INCLUDE_DIRS}")


wombat_package()

add_subdirectory(hbst)
add_subdirectory(srrg2_core)
add_subdirectory(srrg2_solver)

wombat_package_install(
  EXPORT_DEPS
    Eigen3
    OpenCV
)
