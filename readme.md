## CloudTool
A 3D point cloud processing software base on PCL and QT;

![](images/cloudtool.png)

## Depend on
Qt version:     5.15.2  
PCL version:    1.11.1  
VTK version:    8.2.0 (BUILD_WITH_QT) 

## How to build
### Windows
Note: need change -DCMAKE_PREFIX_PATH="QT DIR"
```shell
mkdir build
cd build
cmake .. -DCMAKE_PREFIX_PATH="D:/Qt/5.15.2/msvc2019_64"
cmake --build . --target cloudtool -j 14 --
```
### Ubuntu 20.04
```shell
mkdir build
cd build
cmake .. 
make -j 14
```
