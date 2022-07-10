# CMakeLists files in this project can
# refer to the root source directory of the project as ${TwoScannersMultithreadExample_SOURCE_DIR} and
# to the root binary directory of the project as ${TwoScannersMultithreadExample_BINARY_DIR}.
cmake_minimum_required (VERSION 3.10)

if(POLICY CMP0054)
    cmake_policy(SET CMP0054 NEW)
endif()

project (TwoScannersMultithreadExample)

set(CMAKE_RELEASE_POSTFIX "_Release")
set(CMAKE_DEBUG_POSTFIX "_Debug")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package (Threads REQUIRED)

if (UNIX AND NOT APPLE)
    add_compile_options(-std=c++1y)
    add_compile_options(-pthread)
endif(UNIX AND NOT APPLE)

set(Files
    ${TwoScannersMultithreadExample_SOURCE_DIR}/TwoScannersMultithreadExample.cpp
    ${TwoScannersMultithreadExample_SOURCE_DIR}/ReadMe.txt
)

add_executable (TwoScannersMultithreadExample
    ${Files}
)

find_package(PhoXi REQUIRED CONFIG PATHS "$ENV{PHOXI_CONTROL_PATH}")

if (UNIX AND NOT APPLE)
    target_link_libraries(TwoScannersMultithreadExample
        ${PHOXI_LIBRARY}
        Threads::Threads
        rt
    )
else ()
    target_link_libraries(TwoScannersMultithreadExample
        ${PHOXI_LIBRARY}
        Threads::Threads
    )
endif()

get_target_property(ReleaseBinFolder TwoScannersMultithreadExample RUNTIME_OUTPUT_DIRECTORY_RELEASE)
get_target_property(DebugBinFolder TwoScannersMultithreadExample RUNTIME_OUTPUT_DIRECTORY_DEBUG)

if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    add_custom_command(TARGET TwoScannersMultithreadExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${PHOXI_DLL_DEBUG}"
            $<TARGET_FILE_DIR:TwoScannersMultithreadExample>
    )

    add_custom_command(TARGET TwoScannersMultithreadExample POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different  # which executes "cmake - E copy_if_different..."
            "${PHOXI_DLL_RELEASE}"
            $<TARGET_FILE_DIR:TwoScannersMultithreadExample>
    )
endif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")

target_include_directories (TwoScannersMultithreadExample PUBLIC ${PHOXI_INCLUDE_DIRS})

set_target_properties(TwoScannersMultithreadExample
    PROPERTIES
    #for executables, inheritance of variables CMAKE_DEBUG_POSTFIX... does not work
    DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
    RELEASE_POSTFIX ${CMAKE_RELEASE_POSTFIX}
)
