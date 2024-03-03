file(GLOB LibSrcs *.cpp)

file(GLOB LibHdrs *.h)

add_library(ct_module SHARED ${LibSrcs} ${LibHdrs})

target_link_libraries(ct_module PUBLIC ct_base Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

target_compile_definitions(ct_module PRIVATE CT_LIBRARY)
