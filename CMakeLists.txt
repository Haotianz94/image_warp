CMAKE_MINIMUM_REQUIRED( VERSION 2.8.7 )

SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -g -O3")
PROJECT(image_warp)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${image_warp_SOURCE_DIR}/bin)

SET(PROJECT_HEADER
        ${PROJECT_SOURCE_DIR}/include/grid.h
        ${PROJECT_SOURCE_DIR}/include/mesh.h
#        ${PROJECT_SOURCE_DIR}/include/asap_solver.h
#        ${PROJECT_SOURCE_DIR}/include/image_warper_asap.h
        ${PROJECT_SOURCE_DIR}/include/image_warper_mls.h
        ${PROJECT_SOURCE_DIR}/include/image_warper.h)

SET(PROJECT_SRC
        ${PROJECT_SOURCE_DIR}/src/grid.cpp
        ${PROJECT_SOURCE_DIR}/src/mesh.cpp
#        ${PROJECT_SOURCE_DIR}/src/asap_solver.cpp
#        ${PROJECT_SOURCE_DIR}/src/image_warper_asap.cpp
        ${PROJECT_SOURCE_DIR}/src/image_warper_mls.cpp
        ${PROJECT_SOURCE_DIR}/src/image_warper.cpp
        ${PROJECT_SOURCE_DIR}/src/main.cpp)

INCLUDE_DIRECTORIES(include)

ADD_EXECUTABLE(image_warp ${PROJECT_HEADER} ${PROJECT_SRC})

# opencv
FIND_PACKAGE(OpenCV REQUIRED)
TARGET_LINK_LIBRARIES(image_warp ${OpenCV_LIBS})