## CloudTool version 1.0
A 3D point cloud processing software base on PCL and QT;

## Depend on
Qt version:     5.15.2  
PCL version:    1.11.1  
VTK version:    8.2.0 (BUILD_WITH_QT) 

## How to build
### Windows
1. modify CMAKE_PREFIX_PATH(CMakeLists.txt: line 20)
    ```
    set(CMAKE_PREFIX_PATH "D:/Qt/5.15.2/msvc2019_64")
    ...
    ```
2. configure and build (MSCV)
    ```
    CMD > cmake -S. -B./build -G "Visual Studio 16 2019" -T host=x64 -A x64
        > cmake --build ./build --config Release --target cloudtool -j 14 --
    ```
3. install 
    ```
    CMD > cmake --build ./build --config Release --target install -j 14 --
    ```
