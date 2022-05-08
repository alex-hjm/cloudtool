cmake_minimum_required(VERSION 3.0)

project(CloudTool VERSION 3.0)

if(POLICY CMP0074)
    cmake_policy(SET CMP0074 NEW)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_PREFIX_PATH "D:/Qt/5.15.2/msvc2019_64")
set(CMAKE_PROJECT_INCLUDE_BEFORE D:/Qt/Tools/QtCreator/share/qtcreator/package-manager/auto-setup.cmake)
set(Qt5Core_DIR D:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5Core)
set(Qt5Gui_DIR D:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5Gui)
set(Qt5Widgets_DIR D:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5Widgets)
set(Qt5_DIR D:/Qt/5.15.2/msvc2019_64/lib/cmake/Qt5)

set(CMAKE_CXX_FLAGS_DEFAULT "/DWIN32 /D_WINDOWS /W3 /GR /EHsc")
add_compile_options(/bigobj)
string(APPEND CMAKE_CXX_FLAGS " /MP")

#qt
find_package(QT NAMES Qt6 Qt5 COMPONENTS Widgets REQUIRED)
find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Widgets REQUIRED)
#vtk
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
#pcl
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

if(QT_FOUND)
    message("Found QT")
endif()
if(PCL_FOUND)
    message("Found PCL")
endif()
if(VTK_FOUND)
    message("Found VTK")
endif()

message("Build type: " ${CMAKE_BUILD_TYPE})

include_directories(${PROJECT_SOURCE_DIR})
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin)

add_subdirectory(base) 
add_subdirectory(modules)
