# CMakeLists files in this project can
# refer to the root source directory of the project as ${MinimalOpenCVExample_SOURCE_DIR} and
# to the root binary directory of the project as ${MinimalOpenCVExample_BINARY_DIR}.
cmake_minimum_required (VERSION 3.10)

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

project(MinimalOpenCVExample)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_RELEASE_POSTFIX "_Release")
set(CMAKE_DEBUG_POSTFIX "_Debug")

add_executable(MinimalOpenCVExample
    MinimalOpenCVExample.cpp
)

if(UNIX AND NOT APPLE)
    set(OPEN_CV_PATH "$ENV{HOME}/OpenCV" CACHE PATH "")
else()
    set(OPEN_CV_PATH "C:/opencv/build" CACHE PATH "")
endif()

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
find_package(OpenCV REQUIRED PATHS "${OPEN_CV_PATH}")

target_link_libraries(MinimalOpenCVExample
    ${PHOXI_LIBRARY}
    ${OpenCV_LIBS}
    $<$<PLATFORM_ID:Linux>:rt>
)

if(WIN32)
    string(CONCAT WORLD_VERSION "${OpenCV_VERSION_MAJOR}"
                                "${OpenCV_VERSION_MINOR}"
                                "${OpenCV_VERSION_PATCH}")
    set(OPEN_CV_WORLD_LIB 
        "${OpenCV_LIB_PATH}/../bin/opencv_world${WORLD_VERSION}")
    set(OPEN_CV_RELEASE_DLL_PATH "${OPEN_CV_WORLD_LIB}.dll")
    set(OPEN_CV_DEBUG_DLL_PATH "${OPEN_CV_WORLD_LIB}d.dll")

    add_custom_command(TARGET MinimalOpenCVExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${OPEN_CV_RELEASE_DLL_PATH}"
            $<TARGET_FILE_DIR:MinimalOpenCVExample>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${OPEN_CV_DEBUG_DLL_PATH}"
            $<TARGET_FILE_DIR:MinimalOpenCVExample>
    )

    add_custom_command(TARGET MinimalOpenCVExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${PHOXI_DLL_DEBUG}"
            $<TARGET_FILE_DIR:MinimalOpenCVExample>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${PHOXI_DLL_RELEASE}"
            $<TARGET_FILE_DIR:MinimalOpenCVExample>
    )
endif()

target_include_directories(MinimalOpenCVExample PUBLIC
    ${PHOXI_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
