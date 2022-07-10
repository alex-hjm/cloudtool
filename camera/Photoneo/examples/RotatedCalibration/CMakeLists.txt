cmake_minimum_required (VERSION 3.10)
project(RotatedCalibrationAPIExample)

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

set(CMAKE_RELEASE_POSTFIX "_Release")
set(CMAKE_DEBUG_POSTFIX "_Debug")


if (UNIX AND NOT APPLE)
    add_compile_options(-std=c++1y)
    add_compile_options(-pthread)
endif(UNIX AND NOT APPLE)


set (Files
    RotatedCalibrationAPIExample.cpp
    ReadMe.txt
)

add_executable (RotatedCalibrationAPIExample 
    ${Files}
)

set(CMAKE_AUTOMOC ON)


find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")

if (UNIX AND NOT APPLE)
    target_link_libraries(RotatedCalibrationAPIExample
        ${PHOXI_LIBRARY}
        rt
    )
else (UNIX AND NOT APPLE)
    target_link_libraries(RotatedCalibrationAPIExample
        ${PHOXI_LIBRARY}
    )
endif(UNIX AND NOT APPLE)

get_target_property(ReleaseBinFolder RotatedCalibrationAPIExample RUNTIME_OUTPUT_DIRECTORY_RELEASE)
get_target_property(DebugBinFolder RotatedCalibrationAPIExample RUNTIME_OUTPUT_DIRECTORY_DEBUG)


if(MSVC)
    add_custom_command(TARGET RotatedCalibrationAPIExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${PHOXI_DLL_DEBUG}"
            $<TARGET_FILE_DIR:RotatedCalibrationAPIExample>
    )

    add_custom_command(TARGET RotatedCalibrationAPIExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${PHOXI_DLL_RELEASE}"
            $<TARGET_FILE_DIR:RotatedCalibrationAPIExample>
    )
endif(MSVC)

target_include_directories (RotatedCalibrationAPIExample PUBLIC ${PHOXI_INCLUDE_DIRS})



set_target_properties(RotatedCalibrationAPIExample
    PROPERTIES
    #for executables, inheritance of variables CMAKE_DEBUG_POSTFIX... does not work
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)