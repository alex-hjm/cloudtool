# set up sources to build
set(LibSrcs
        cloud.cpp
        common.cpp
        # cloudview.cpp
        
        # customtree.cpp
        # cloudtree.cpp
        # console.cpp
        # fileio.cpp
        # features.cpp
        # filters.cpp
        # keypoints.cpp
        # recognition.cpp
        # registration.cpp
        # segmentation.cpp
        # surface.cpp
        )

set(MocHeaders
        # cloudview.h
        # customtree.h
        # cloudtree.h
        # console.h
        # customdock.h
        # customdialog.h
        # fileio.h
        # features.h
        # filters.h
        # keypoints.h
        # recognition.h
        # registration.h
        # segmentation.h
        # surface.h
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

target_link_libraries(ct_base PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)
target_link_libraries(ct_base PRIVATE ${PCL_LIBRARIES} ${VTK_LIBRARIES})
target_compile_definitions(ct_base PRIVATE CT_LIBRARY)
