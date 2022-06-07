set(LibSrcs
        )

set(MocHeaders
        )

set(NonMocHeaders
        )

qt5_wrap_cpp(LibMocSrcs ${MocHeaders})

add_library(ct_modules SHARED ${LibSrcs} ${LibMocSrcs} ${NonMocHeaders} ${MocHeaders})

set(LibHeaders ${NonMocHeaders} ${MocHeaders})

set_target_properties(ct_modules PROPERTIES PUBLIC_HEADER "${LibHeaders}")

target_link_libraries(ct_modules PRIVATE ct_base Qt${QT_VERSION_MAJOR}::Widgets 
                                         ${PCL_LIBRARIES} ${VTK_LIBRARIES})
target_compile_definitions(ct_modules PRIVATE CT_LIBRARY)
