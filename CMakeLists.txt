cmake_minimum_required(VERSION 3.0)

project(CloudTool VERSION 2.0)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(POLICY CMP0074)
  cmake_policy(SET CMP0074 NEW)
endif()

message(STATUS "Build type: " ${CMAKE_BUILD_TYPE})

 if(MSVC)
  set(CMAKE_COMPILER_IS_MSVC 1)
elseif(MINGW)
  set(CMAKE_COMPILER_IS_MINGW 1)
endif()

if(CMAKE_COMPILER_IS_MSVC)
  set(CMAKE_CXX_FLAGS_DEFAULT "/DWIN32 /D_WINDOWS /W3 /GR /EHsc")
else()
  set(CMAKE_CXX_FLAGS_DEFAULT "")
endif()

if(CMAKE_COMPILER_IS_GNUCXX)
  if("${CMAKE_CXX_FLAGS}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
    if (CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7)
      string(APPEND CMAKE_CXX_FLAGS " -Wabi=11")
    else()
      string(APPEND CMAKE_CXX_FLAGS " -Wabi")
    endif()
    string(APPEND CMAKE_CXX_FLAGS " -Wall -Wextra -fno-strict-aliasing ${SSE_FLAGS} ${AVX_FLAGS}")
  endif()

  if("${CMAKE_SHARED_LINKER_FLAGS}" STREQUAL "" AND NOT CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--as-needed")
  endif()
endif()

if(CMAKE_COMPILER_IS_MSVC)
  add_definitions("-DBOOST_ALL_NO_LIB -D_SCL_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_WARNINGS -DNOMINMAX ${SSE_DEFINITIONS}")
  
  if("${CMAKE_CXX_FLAGS}" STREQUAL "${CMAKE_CXX_FLAGS_DEFAULT}")
    string(APPEND CMAKE_CXX_FLAGS " /fp:precise /bigobj /wd4996 /source-charset:utf-8 ${SSE_FLAGS} ${AVX_FLAGS}")
    string(APPEND CMAKE_CXX_FLAGS " /wd4800 /wd4521 /wd4251 /wd4275 /wd4305 /wd4355 /w34265")

    include(ProcessorCount)
    ProcessorCount(CPUCores)
    set(MSVC_MP ${CPUCores})
    if (CMAKE_VERSION VERSION_LESS 3.11.0)
      if(MSVC_MP EQUAL 0)
        string(APPEND CMAKE_C_FLAGS " /MP")
        string(APPEND CMAKE_CXX_FLAGS " /MP")
      elseif(MSVC_MP GREATER 1)
        string(APPEND CMAKE_C_FLAGS " /MP${MSVC_MP}")
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
  string(APPEND CMAKE_CXX_FLAGS " /bigobj")

  if(CMAKE_GENERATOR STREQUAL "Ninja")
    string(APPEND CMAKE_C_FLAGS " /FS")
    string(APPEND CMAKE_CXX_FLAGS " /FS")
  endif()
endif()

# OpenMP
find_package(OpenMP COMPONENTS C CXX)
if(OpenMP_FOUND)
  string(APPEND CMAKE_C_FLAGS " ${OpenMP_C_FLAGS}")
  string(APPEND CMAKE_CXX_FLAGS " ${OpenMP_CXX_FLAGS}")

  message(STATUS "Found OpenMP, spec date ${OpenMP_CXX_SPEC_DATE}")

  if((MSVC_VERSION EQUAL 1900) OR (MSVC_VERSION MATCHES "^191[0-9]$"))
    string(APPEND CMAKE_SHARED_LINKER_FLAGS_DEBUG " /DELAYLOAD:VCOMP140D.dll")
    string(APPEND CMAKE_SHARED_LINKER_FLAGS_RELEASE " /DELAYLOAD:VCOMP140.dll")
  endif()
else()
  message(STATUS "Not found OpenMP")
endif()

#PCL
find_package(PCL REQUIRED)
if(PCL_FOUND)
    include_directories(${PCL_INCLUDE_DIRS})
    link_directories(${PCL_LIBRARY_DIRS})
    add_definitions(${PCL_DEFINITIONS})
    message(STATUS "Found PCL, version is ${PCL_VERSION}.")
else()
    message(STATUS "Not found PCL")
endif()

#VTK
find_package(VTK REQUIRED)
if(VTK_FOUND)
    include_directories(${VTK_INCLUDE_DIRS})
    link_directories(${VTK_LIBRARIES})
    message(STATUS "Found VTK, version is ${VTK_VERSION}.")
else()
    message(STATUS "Not found VTK")
endif()

# QT
find_package(QT NAMES Qt5 COMPONENTS Concurrent Core Widgets OpenGL Gui LinguistTools REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets LinguistTools REQUIRED)

if(QT_FOUND)
    message(STATUS "Found QT, version is ${Qt5_VERSION}.")
else()
    message(STATUS "Not found QT.")
endif()

include_directories(${PROJECT_SOURCE_DIR})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

add_subdirectory(base)
add_subdirectory(test)