set(LibSrcs
        cloud.cpp
        common.cpp
        fileio.cpp
        console.cpp
        cloudview.cpp
        customtree.cpp
        cloudtree.cpp
        )

set(LibHdrs
        exports.h
        cloud.h
        common.h
        fileio.h
        console.h
        cloudview.h
        customtree.h
        cloudtree.h
        customdock.h
        customdialog.h
        )

find_package(VTK COMPONENTS
        vtkGUISupportQt
        vtkGUISupportQtOpenGL
        vtkRenderingGL2PSOpenGL2
        vtkRenderingVolumeOpenGL2)

add_library(ct_base SHARED ${LibSrcs} ${LibHdrs})

target_link_libraries(ct_base PRIVATE Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

target_compile_definitions(ct_base PRIVATE CT_LIBRARY)

set_target_properties(ct_base PROPERTIES PUBLIC_HEADER "${LibHdrs}")

install(TARGETS ct_base PUBLIC_HEADER DESTINATION include/base)