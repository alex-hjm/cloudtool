file(GLOB LibSrcs *.cpp)

file(GLOB LibHdrs *.h)

add_library(ct_base SHARED ${LibSrcs} ${LibHdrs})

target_link_libraries(ct_base PRIVATE Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

target_compile_definitions(ct_base PRIVATE CT_LIBRARY)

set_target_properties(ct_base PROPERTIES PUBLIC_HEADER "${LibHdrs}")

add_subdirectory(test)
