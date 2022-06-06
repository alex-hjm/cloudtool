set(LibSrcs
        cloud.cpp
        common.cpp
        fileio.cpp
        console.cpp
        cloudview.cpp
        customtree.cpp
        cloudtree.cpp
        )

set(MocHeaders
        fileio.h
        console.h
        cloudview.h
        customtree.h
        cloudtree.h
        customdock.h
        customdialog.h
        )

set(NonMocHeaders
        exports.h
        cloud.h
        common.h
        )

find_package(VTK COMPONENTS
        vtkGUISupportQt
        vtkGUISupportQtOpenGL
        vtkRenderingGL2PSOpenGL2
        vtkRenderingVolumeOpenGL2)

qt5_wrap_cpp(LibMocSrcs ${MocHeaders})

add_library(ct_base SHARED ${LibSrcs} ${LibMocSrcs} ${NonMocHeaders} ${MocHeaders})

set(LibHeaders ${NonMocHeaders} ${MocHeaders})

set_target_properties(ct_base PROPERTIES PUBLIC_HEADER "${LibHeaders}")

target_link_libraries(ct_base PRIVATE Qt${QT_VERSION_MAJOR}::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})
target_compile_definitions(ct_base PRIVATE CT_LIBRARY)
