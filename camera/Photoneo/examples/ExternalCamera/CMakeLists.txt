cmake_minimum_required (VERSION 3.10)

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

project (ExternalCameraExample)

set(CMAKE_RELEASE_POSTFIX "_Release")
set(CMAKE_DEBUG_POSTFIX "_Debug")

if (UNIX AND NOT APPLE)
    add_compile_options(-std=c++1y)
    add_compile_options(-pthread)
endif(UNIX AND NOT APPLE)

set(Settings_DIR "${ExternalCameraExample_SOURCE_DIR}/Settings")
set(Data_DIR "${ExternalCameraExample_SOURCE_DIR}/Data")

file(COPY ${Settings_DIR}/
DESTINATION ${ExternalCameraExample_BINARY_DIR}/Settings/)
file(COPY ${Data_DIR}/
DESTINATION ${ExternalCameraExample_BINARY_DIR}/Data/)

set(Files
    ${ExternalCameraExample_SOURCE_DIR}/ExternalCameraExample.cpp
    ${ExternalCameraExample_SOURCE_DIR}/ReadMe.txt
)

add_executable (ExternalCameraExample 
    ${Files}
)

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")

if (UNIX AND NOT APPLE)
    set(OPEN_CV_PATH "$ENV{HOME}/OpenCV")
else ()
    set(OPEN_CV_PATH "C:/opencv/build")
endif ()

find_package(OpenCV REQUIRED PATHS "${OPEN_CV_PATH}" )
target_link_libraries(ExternalCameraExample ${OpenCV_LIBS})

if (UNIX AND NOT APPLE)
    target_link_libraries(ExternalCameraExample ${PHOXI_LIBRARY})
else (UNIX AND NOT APPLE)
    target_link_libraries(ExternalCameraExample ${PHOXI_LIBRARY})
endif(UNIX AND NOT APPLE)

get_target_property(ReleaseBinFolder ExternalCameraExample RUNTIME_OUTPUT_DIRECTORY_RELEASE)
get_target_property(DebugBinFolder ExternalCameraExample RUNTIME_OUTPUT_DIRECTORY_DEBUG)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    if(MSVC_TOOLSET_VERSION GREATER_EQUAL 140)
        set(PHO_COMPILER_VERSION "vc14")
    elseif(MSVC_TOOLSET_VERSION EQUAL 120)
        set(PHO_COMPILER_VERSION "vc12")
    endif()
    
    set(OPEN_CV_RELEASE_DLL_PATH "${OPEN_CV_PATH}/x64/${PHO_COMPILER_VERSION}/bin/opencv_world310.dll")
    set(OPEN_CV_DEBUG_DLL_PATH "${OPEN_CV_PATH}/x64/${PHO_COMPILER_VERSION}/bin/opencv_world310d.dll")
    
    add_custom_command(TARGET ExternalCameraExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${OPEN_CV_RELEASE_DLL_PATH}"
            $<TARGET_FILE_DIR:ExternalCameraExample>
    )
    add_custom_command(TARGET ExternalCameraExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${OPEN_CV_DEBUG_DLL_PATH}"
            $<TARGET_FILE_DIR:ExternalCameraExample>
    )
    add_custom_command(TARGET ExternalCameraExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${PHOXI_DLL_DEBUG}"
            $<TARGET_FILE_DIR:ExternalCameraExample>
    )
    add_custom_command(TARGET ExternalCameraExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${PHOXI_DLL_RELEASE}"
            $<TARGET_FILE_DIR:ExternalCameraExample>
    )
endif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")

target_include_directories(ExternalCameraExample PUBLIC ${PHOXI_INCLUDE_DIRS})

set_target_properties(ExternalCameraExample
    PROPERTIES
    #for executables, inheritance of variables CMAKE_DEBUG_POSTFIX... does not work
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)