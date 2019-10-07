project(image_warp)

#**************************************************************************************************
# General cMake settings
#**************************************************************************************************
cmake_minimum_required(VERSION 3.5)

#**************************************************************************************************
# Find Package **************************************************************************************************
find_package(OpenCV REQUIRED)
MESSAGE( " *** OpenCV_INCLUDE_DIRS : " ${OpenCV_INCLUDE_DIRS} )
MESSAGE( " *** OpenCV_LIB_DIRS : " ${OpenCV_LIB_DIRS} )
MESSAGE( " *** OpenCV_LIBS : " ${OpenCV_LIBS} )

find_package(PythonLibs 3 REQUIRED)
MESSAGE( " *** PYTHON_INCLUDE_DIRS : " ${PYTHON_INCLUDE_DIRS} )
MESSAGE( " *** PYTHON_LIBRARIES : " ${PYTHON_LIBRARIES} )

find_package(pybind11 CONFIG REQUIRED)

#**************************************************************************************************
# Include **************************************************************************************************
include_directories(${PYTHON_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS}/opencv4)
include_directories(${pybind11_INCLUDE_DIR})
include_directories(include)

#**************************************************************************************************
# Set variable **************************************************************************************************
SET(SOURCES
	${PROJECT_SOURCE_DIR}/src/grid.cpp
	${PROJECT_SOURCE_DIR}/src/mesh.cpp
#    ${PROJECT_SOURCE_DIR}/src/asap_solver.cpp
#    ${PROJECT_SOURCE_DIR}/src/image_warper_asap.cpp
	${PROJECT_SOURCE_DIR}/src/image_warper_mls.cpp
	${PROJECT_SOURCE_DIR}/src/image_warper.cpp
	${PROJECT_SOURCE_DIR}/src/python_wrap.cpp
	${PROJECT_SOURCE_DIR}/src/ndarray_converter.cpp
)

#**************************************************************************************************
# Set compiler **************************************************************************************************
SET(CMAKE_CXX_FLAGS "-std=c++11 -Wall -O3 -fPIC")

#**************************************************************************************************
# Linker **************************************************************************************************
LINK_DIRECTORIES(
  ${OpenCV_LIB_DIR}
)

#**************************************************************************************************
# Make configuration
#**************************************************************************************************
add_library(image_warp SHARED ${SOURCES})
target_link_libraries(image_warp ${PYTHON_LIBRARIES} ${OpenCV_LIBS})
SET_TARGET_PROPERTIES(image_warp PROPERTIES PREFIX "")

install(TARGETS image_warp DESTINATION ~/.local/lib/python3.6/site-packages/)

