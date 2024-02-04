file(GLOB LibSrcs *.cpp)

file(GLOB LibHdrs *.h)

add_executable(test_cloudlist WIN32 ${LibSrcs} ${LibHdrs})

target_link_libraries(test_cloudlist  PRIVATE ct_base Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

