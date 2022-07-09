cmake_minimum_required(VERSION 3.0)

project(CloudTool VERSION 3.0)

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if("${CMAKE_BUILD_TYPE}" STREQUAL "")
  set(CMAKE_BUILD_TYPE Release)
endif()

if(WIN32)
    set(CMAKE_PREFIX_PATH "D:/Qt/5.15.2/msvc2019_64")
    set(Qt5Core_DIR D:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5Core)
    set(Qt5Gui_DIR D:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5Gui)
    set(Qt5Widgets_DIR D:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5Widgets)
    set(Qt5_DIR D:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5)
endif()

if(MSVC)
    set(CMAKE_CXX_FLAGS_DEFAULT "/DWIN32 /D_WINDOWS /W3 /GR /EHsc")

    if("${CMAKE_CXX_FLAGS}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
        string(APPEND CMAKE_CXX_FLAGS " /fp:precise /bigobj /wd4996 /source-charset:utf-8 ")
        set(MSVC_MP ${CPUCores})

        if(CMAKE_VERSION VERSION_LESS 3.11.0)
            if(MSVC_MP EQUAL 0)
                string(APPEND CMAKE_CXX_FLAGS " /MP")
            elseif(MSVC_MP GREATER 1)
                string(APPEND CMAKE_CXX_FLAGS " /MP${MSVC_MP}")
            endif()
        else()
            if(MSVC_MP EQUAL 0)
                add_compile_options($<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:/MP>)
            elseif(MSVC_MP GREATER 1)
                add_compile_options($<$<OR:$<COMPILE_LANGUAGE:C>,$<COMPILE_LANGUAGE:CXX>>:/MP${MSVC_MP}>)
            endif()
        endif()
    endif()

    if(CMAKE_GENERATOR STREQUAL "Ninja")
        string(APPEND CMAKE_C_FLAGS " /FS")
        string(APPEND CMAKE_CXX_FLAGS " /FS")
    endif()
elseif(CMAKE_COMPILER_IS_GNUCXX)
    set(CMAKE_CXX_FLAGS_DEFAULT "")

    if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wabi=11")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wabi")
    endif()

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-unknown-pragmas -fno-strict-aliasing"
        "-Wno-format-extra-args -Wno-sign-compare -Wno-invalid-offsetof -Wno-conversion ${SSE_FLAGS_STR}")
endif()

# OpenMP (optional)
option(WITH_OPENMP "Build with parallelization using OpenMP" TRUE)

if(WITH_OPENMP)
    find_package(OpenMP COMPONENTS C CXX)
endif()

if(OpenMP_FOUND)
    string(APPEND CMAKE_C_FLAGS " ${OpenMP_C_FLAGS}")
    string(APPEND CMAKE_CXX_FLAGS " ${OpenMP_CXX_FLAGS}")

    if(${CMAKE_VERSION} VERSION_LESS "3.7")
        message(STATUS "Found OpenMP")
    else()
        message(STATUS "Found OpenMP, spec date ${OpenMP_CXX_SPEC_DATE}")
    endif()

    if(MSVC)
        if(MSVC_VERSION EQUAL 1900)
            set(OPENMP_DLL VCOMP140)
        elseif(MSVC_VERSION MATCHES "^191[0-9]$")
            set(OPENMP_DLL VCOMP140)
        elseif(MSVC_VERSION MATCHES "^192[0-9]$")
            set(OPENMP_DLL VCOMP140)
        endif()

        if(OPENMP_DLL)
            string(APPEND CMAKE_SHARED_LINKER_FLAGS_DEBUG " /DELAYLOAD:${OPENMP_DLL}D.dll")
            string(APPEND CMAKE_SHARED_LINKER_FLAGS_RELEASE " /DELAYLOAD:${OPENMP_DLL}.dll")
        else()
            message(WARNING "Delay loading flag for OpenMP DLL is invalid.")
        endif()
    endif()
else()
    message(STATUS "Not found OpenMP")
endif()

# qt
find_package(QT NAMES Qt5 COMPONENTS Core Widgets OpenGL Gui REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)

if(QT_FOUND)
    message(STATUS "Found QT version: ${Qt5_VERSION}.")
else()
    message(STATUS "Qt5 is not found.")
endif()

# vtk
find_package(VTK 8.2.0 REQUIRED)

if(VTK_FOUND)
    include(${VTK_USE_FILE})
    message(STATUS "Found VTK version: ${VTK_VERSION}.")
else()
    message(STATUS "VTK is not found.")
endif()

# pcl
find_package(PCL 1.11.1 REQUIRED)

if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    message(STATUS "Found PCL version: ${PCL_VERSION}.")
else()
    message(STATUS "PCL is not found.")
endif()

# camera sdk (only support windows)
if(WIN32)
    # 1. Azure Kinect
    set(AzureKinect_DIR "${PROJECT_SOURCE_DIR}/camera/AzureKinect")
    include_directories("${AzureKinect_DIR}/include")
    link_directories("${AzureKinect_DIR}/lib")
    # 2. Photoneo SDK
    set(Photoneo_DIR "${PROJECT_SOURCE_DIR}/camera/Photoneo")
    include_directories("${Photoneo_DIR}/include")
    link_directories("${Photoneo_DIR}/lib")
endif()

message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

include_directories(${PROJECT_SOURCE_DIR})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)
set(CMAKE_INSTALL_PREFIX ${PROJECT_SOURCE_DIR}/install)

add_definitions(-DROOT_PATH="${PROJECT_SOURCE_DIR}")
add_definitions(-DDATA_PATH="${PROJECT_SOURCE_DIR}/data/")

add_subdirectory(base)
add_subdirectory(modules)
add_subdirectory(cloudtool)
