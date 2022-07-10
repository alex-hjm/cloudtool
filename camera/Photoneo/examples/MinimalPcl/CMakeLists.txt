# CMakeLists files in this project can
# refer to the root source directory of the project as ${MinimalPclExample_SOURCE_DIR} and
# to the root binary directory of the project as ${MinimalPclExample_BINARY_DIR}.
cmake_minimum_required (VERSION 3.10)

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

project (MinimalPclExample)

set(CMAKE_RELEASE_POSTFIX "_Release")
set(CMAKE_DEBUG_POSTFIX "_Debug")


if (UNIX AND NOT APPLE)
    add_compile_options(-std=c++1y)
    add_compile_options(-pthread)
endif(UNIX AND NOT APPLE)

set(Files
    ${MinimalPclExample_SOURCE_DIR}/MinimalPclExample.cpp
    ${MinimalPclExample_SOURCE_DIR}/ReadMe.txt
)

add_executable (MinimalPclExample
    ${Files}
)

# set(ThirdPartyDIR "/path/to/your/3rd/party/libraries/directory")

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")
find_package(PCL 1.8.0 EXACT PATHS "${ThirdPartyDIR}/pcl-1.8.0/cmake" "${ThirdPartyDIR}/pcl-1.8.0/share/pcl-1.8")

if (UNIX AND NOT APPLE)
    target_link_libraries(MinimalPclExample
        ${PHOXI_LIBRARY}
        rt
    )
else (UNIX AND NOT APPLE)
    target_link_libraries(MinimalPclExample
        ${PHOXI_LIBRARY}
    )
endif(UNIX AND NOT APPLE)

get_target_property(ReleaseBinFolder MinimalPclExample RUNTIME_OUTPUT_DIRECTORY_RELEASE)
get_target_property(DebugBinFolder MinimalPclExample RUNTIME_OUTPUT_DIRECTORY_DEBUG)

if(MSVC)
    add_custom_command(TARGET MinimalPclExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${PHOXI_DLL_DEBUG}"
            $<TARGET_FILE_DIR:MinimalPclExample>
    )

    add_custom_command(TARGET MinimalPclExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${PHOXI_DLL_RELEASE}"
            $<TARGET_FILE_DIR:MinimalPclExample>
    )
endif(MSVC)

target_include_directories (MinimalPclExample PUBLIC ${PHOXI_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})

set_target_properties(MinimalPclExample
    PROPERTIES
    #for executables, inheritance of variables CMAKE_DEBUG_POSTFIX... does not work
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)
