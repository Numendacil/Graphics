CMAKE_MINIMUM_REQUIRED(VERSION 2.8)
PROJECT(PM CXX)

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

ADD_SUBDIRECTORY(deps/vecmath)

SET(PM_SOURCES
        src/image.cpp
        src/main.cpp
        src/mesh.cpp
        src/scene_parser.cpp
        src/render.cpp
        src/utils.cpp)

SET(PM_INCLUDES
        include/camera.hpp
        include/group.hpp
        include/hit.hpp
        include/image.hpp
        include/light.hpp
        include/material.hpp
        include/mesh.hpp
        include/object3d.hpp
        include/plane.hpp
        include/ray.hpp
        include/scene_parser.hpp
        include/sphere.hpp
        include/transform.hpp
        include/triangle.hpp
        include/utils.hpp
        include/photon_map.hpp
        include/render.hpp
        )

SET(CMAKE_CXX_STANDARD 17)
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

ADD_EXECUTABLE(${PROJECT_NAME} ${PM_SOURCES} ${PM_INCLUDES})
TARGET_LINK_LIBRARIES(${PROJECT_NAME} vecmath)
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PRIVATE include)
