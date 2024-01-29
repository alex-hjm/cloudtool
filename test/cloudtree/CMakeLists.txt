file(GLOB LibSrcs *.cpp)

file(GLOB LibHdrs *.h)

add_executable(test_cloudtree WIN32 ${LibSrcs} ${LibHdrs})

target_link_libraries(test_cloudtree  PRIVATE ct_base Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

