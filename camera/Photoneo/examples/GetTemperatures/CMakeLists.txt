cmake_minimum_required (VERSION 3.10)

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

project(GetTemperatures)

set(CMAKE_RELEASE_POSTFIX "_Release")
set(CMAKE_DEBUG_POSTFIX "_Debug")

if (UNIX AND NOT APPLE)
    add_compile_options(-std=c++1y)
    add_compile_options(-pthread)
endif(UNIX AND NOT APPLE)

set(Files
    ${GetTemperatures_SOURCE_DIR}/GetTemperaturesExample.cpp
    ${GetTemperatures_SOURCE_DIR}/ReadMe.txt
)

add_executable(GetTemperatures
    ${Files}
)

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")

if(UNIX AND NOT APPLE)
    target_link_libraries(GetTemperatures
        ${PHOXI_LIBRARY}
        rt
    )
else()
    target_link_libraries(GetTemperatures
        ${PHOXI_LIBRARY}
    )
endif()

if(MSVC)
    add_custom_command(TARGET GetTemperatures POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
            $<$<CONFIG:Release>:${PHOXI_DLL_RELEASE}>
            $<$<CONFIG:Debug>:${PHOXI_DLL_DEBUG}>
            $<TARGET_FILE_DIR:GetTemperatures>
    )
endif()

target_include_directories(GetTemperatures PUBLIC 
    ${PHOXI_INCLUDE_DIRS}
)