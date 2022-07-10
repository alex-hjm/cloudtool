# CMakeLists files in this project can
# refer to the root source directory of the project as ${ApplyCustomProjectionExample_SOURCE_DIR} and
# to the root binary directory of the project as ${ApplyCustomProjectionExample_BINARY_DIR}.
cmake_minimum_required(VERSION 3.10)

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

project(ApplyCustomProjectionExample)

set(CMAKE_RELEASE_POSTFIX "_Release")
set(CMAKE_DEBUG_POSTFIX "_Debug")

if(UNIX AND NOT APPLE)
    add_compile_options(-std=c++1y)
    add_compile_options(-pthread)
endif(UNIX AND NOT APPLE)

set(Files
    ${ApplyCustomProjectionExample_SOURCE_DIR}/ApplyCustomProjectionExample.cpp
    ${ApplyCustomProjectionExample_SOURCE_DIR}/ReadMe.txt
)

add_executable(ApplyCustomProjectionExample
    ${Files}
)

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")

if(UNIX AND NOT APPLE)
    set(OPEN_CV_PATH "$ENV{HOME}/OpenCV" CACHE PATH "")
else()
    set(OPEN_CV_PATH "C:/opencv/build" CACHE PATH "")
endif()

find_package(OpenCV REQUIRED PATHS "${OPEN_CV_PATH}")
target_link_libraries(ApplyCustomProjectionExample
    ${OpenCV_LIBS}
)
set(OPEN_CV_BIN_PATH "${OpenCV_LIB_DIR_OPT}/../bin")
 
if(UNIX AND NOT APPLE)
    target_link_libraries(ApplyCustomProjectionExample
        ${PHOXI_LIBRARY}
        rt
    )
else()
    target_link_libraries(ApplyCustomProjectionExample
        ${PHOXI_LIBRARY}
    )
endif()

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    set(OPEN_CV_RELEASE_DLL_PATH "${OPEN_CV_BIN_PATH}/opencv_world310.dll")
    set(OPEN_CV_DEBUG_DLL_PATH "${OPEN_CV_BIN_PATH}/opencv_world310d.dll")
    add_custom_command(TARGET ApplyCustomProjectionExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${OPEN_CV_RELEASE_DLL_PATH}"
            $<TARGET_FILE_DIR:ApplyCustomProjectionExample>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${OPEN_CV_DEBUG_DLL_PATH}"
            $<TARGET_FILE_DIR:ApplyCustomProjectionExample>
    )

    add_custom_command(TARGET ApplyCustomProjectionExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${PHOXI_DLL_DEBUG}"
            $<TARGET_FILE_DIR:ApplyCustomProjectionExample>
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "${PHOXI_DLL_RELEASE}"
            $<TARGET_FILE_DIR:ApplyCustomProjectionExample>
    )
endif()

target_include_directories(ApplyCustomProjectionExample PUBLIC
    ${PHOXI_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)
